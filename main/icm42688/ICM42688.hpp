// version: 1.0.0
#pragma once

#ifndef ICM_H
#define ICM_H
#include "SPICREATE.hpp" // 3.0.0
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

#define POWER_MANAGEMENT 0x4E
#define WHO_AM_I_Address 0x75
#define ICM_Data_Adress 0x1F
#define INT_CONFIG1 0x64  // Bank0 内部クロック設定
#define INT_CONFIG5 0x7B  // Bank1 INT2ルーティング
#define REG_BANK_SEL 0x76 // Bank変更

class ICM
{
    int CS;
    int deviceHandle{-1};
    SPICreate *ICMSPI;

public:
    void begin(SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoAmI();
    uint8_t UserBank();
    void Get(int16_t *rx);
};

void ICM::begin(SPICreate *targetSPI, int cs, uint32_t freq)
{
    CS = cs;
    ICMSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = 3; // 0 or 3
    if_cfg.queue_size = 1;

    deviceHandle = ICMSPI->addDevice(&if_cfg, cs);

    // 1) PWR_MGMT0 (0x4E) に 0x00 を書き込み：センサーを全OFF
    //    - [データシート] "14.36 PWR_MGMT0" (p.77)
    //      bit[3:2]=00(GYRO=OFF), bit[1:0]=00(ACCEL=OFF), TEMP_DIS=0
    //    - センサーをOFFにしてからクロック周りを設定するための事前処理
    ICMSPI->setReg(POWER_MANAGEMENT, 0x00, deviceHandle); // リブート対策

    // 2) 200us待機
    //    - [データシート] PWR_MGMT0の備考 (p.77)
    //      「OFF→他モードへの切り替え時は200µs間レジスタ書き込みを行わない」推奨がある
    esp_rom_delay_us(200); // 200us間はレジスタ変更禁止

    // 3) INTF_CONFIG1 (0x4D) に 0x01 を書き込み
    //    - [データシート] "14.35 INTF_CONFIG1" (p.76)
    //      bit[1:0]=01: “PLLが使用可能ならPLL、なければRC” → CLKSEL=01
    //      bit2=0: RTC_MODE=0(外部クロックはまだ必須にしない)
    //    - まずはCLKSELだけ 01(PLL優先) にしておく設定
    ICMSPI->setReg(INT_CONFIG1, 0x01, deviceHandle);

    // 4) REG_BANK_SEL (0x76) に 0x01：ユーザバンク1へ切り替え
    //    - [データシート] "17.19 REG_BANK_SEL" (p.89)
    //      bit[2:0] = 001 → Bank1
    ICMSPI->setReg(REG_BANK_SEL, 0x01, deviceHandle);

    // 5) INTF_CONFIG5 (0x7B, Bank1) に 0x04 を書き込み
    //    - [データシート] "15.18 INTF_CONFIG5" (p.94)
    //      bit2:1 = 10 → PIN9_FUNCTION=CLKIN
    //      PIN9を外部クロック入力ピンとして使用する設定
    ICMSPI->setReg(INT_CONFIG5, 0x04, deviceHandle);

    // 6) REG_BANK_SEL (0x76) に 0x00：バンク0へ戻す
    //    - [同上] "17.19 REG_BANK_SEL" (p.89)
    //      bit[2:0] = 000 → Bank0
    ICMSPI->setReg(REG_BANK_SEL, 0x00, deviceHandle);

    // 7) INTF_CONFIG1 (0x4D) に 0x05 を書き込み
    //    - [データシート] "14.35 INTF_CONFIG1" (p.76)
    //      0x05 = 0b0101 → bit2=1 (RTC_MODE=1: 外部クロック必須), bit[1:0]=01 (PLL優先)
    //      これで本格的に外部CLKIN + PLL使用モードになる
    ICMSPI->setReg(INT_CONFIG1, 0x05, deviceHandle);

    // 8) PWR_MGMT0 (0x4E) に 0x0F を書き込み
    //    - [データシート] "14.36 PWR_MGMT0" (p.77)
    //      0x0F = 0b00001111 → bit[3:2]=11(GYRO=LowNoise), bit[1:0]=11(ACCEL=LowNoise)
    //      ジャイロ・加速度計をLNモードで有効化
    ICMSPI->setReg(POWER_MANAGEMENT, 0x0F, deviceHandle);

    // 9) 45ms待機
    //    - [データシート] "14.36 PWR_MGMT0" (p.77) 備考
    //      「GyroをONにした後、安定化のために最低45msは必要」と言及あり
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount(); // 現在のTickを取得
    vTaskDelayUntil(&xLastWakeTime, 45 / portTICK_PERIOD_MS);
    return;
}
uint8_t ICM::WhoAmI()
{
    return ICMSPI->readByte(0x80 | WHO_AM_I_Address, deviceHandle);
}

/**
 * @fn
 * ICMから加速度、角速度を取得
 */
void ICM::Get(int16_t *rx)
{
    uint8_t rx_buf[12];
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (12) * 8;
    comm.cmd = ICM_Data_Adress | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    ICMSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);

    rx[0] = (rx_buf[0] << 8 | rx_buf[1]);
    rx[1] = (rx_buf[2] << 8 | rx_buf[3]);
    rx[2] = (rx_buf[4] << 8 | rx_buf[5]);
    rx[3] = (rx_buf[6] << 8 | rx_buf[7]);
    rx[4] = (rx_buf[8] << 8 | rx_buf[9]);
    rx[5] = (rx_buf[10] << 8 | rx_buf[11]);
    return;
}
#endif