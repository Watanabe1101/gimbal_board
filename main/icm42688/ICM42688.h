#pragma once

#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "SPICREATE.h"

// ICM42688のレジスタ定義例（実際のデータシートを参照してください）
#define ICM42688_REG_POWER_MGMT    0x4E
#define ICM42688_REG_WHO_AM_I      0x75
#define ICM42688_REG_ACCEL_GYRO    0x1F  // 仮: 加速度・ジャイロ一括読み取り口

class ICM42688
{
public:
    ICM42688() : _spi(nullptr), _handle(nullptr) {}

    /**
     * @brief 初期化
     * @param spi     SPICreate のインスタンス
     * @param csPin   CSピン番号
     * @param freq    SPI通信周波数(Hz)
     */
    esp_err_t begin(SPICreate *spi, gpio_num_t csPin, uint32_t freq)
    {
        if (!spi) return ESP_ERR_INVALID_ARG;
        _spi = spi;

        // spi_device_interface_config_t 設定
        spi_device_interface_config_t devcfg = {};
        devcfg.command_bits = 0;        // 可変コマンド位数の必要があれば設定
        devcfg.address_bits = 0;        // 可変アドレス位数の必要があれば設定
        devcfg.dummy_bits = 0;         // ダミーサイクル不要なら0
        devcfg.mode = 0;               // CPOL=0, CPHA=0 => モード0
        devcfg.clock_speed_hz = freq;  // 周波数
        devcfg.spics_io_num = -1;      // ソフト制御のため -1
        devcfg.queue_size = 1;         // キューサイズ
        devcfg.pre_cb = csReset;       // コールバック(事前にcsPin LOW)
        devcfg.post_cb = csSet;        // コールバック(後にcsPin HIGH)
        // devcfg.flags = 0;           // 必要に応じて SPI_DEVICE_HALFDUPLEX等

        // デバイス追加
        _handle = _spi->addDevice(&devcfg, csPin);
        if (!_handle)
        {
            ESP_LOGE(TAG, "Failed to add ICM42688 device");
            return ESP_FAIL;
        }

        // 電源ONなど初期設定(例)
        _spi->writeByte(ICM42688_REG_POWER_MGMT, 0x0F, _handle);

        ESP_LOGI(TAG, "ICM42688 init done");
        return ESP_OK;
    }

    /**
     * @brief WHO_AM_I レジスタを読む
     */
    uint8_t whoAmI()
    {
        if (!_handle) return 0;
        // ICM42688の読み出し時、上位bitに0x80をORする場合もある
        // ここでは仮に 0x80付ける例とする
        return _spi->readByte(ICM42688_REG_WHO_AM_I | 0x80, _handle);
    }

    /**
     * @brief 加速度 & ジャイロ取得 (6軸分)
     * @param[out] out  要素数6以上のint16_t配列
     */
    void getAccelGyro(int16_t *out)
    {
        if (!_handle) return;

        // 例: 加速度XYZ + ジャイロXYZ = 6軸分(1軸あたり2バイト=>12バイト)
        uint8_t rx_buf[12] = {0};
        // 先頭1バイトをアドレスとして送る必要があるならば、transmitで対応
        // or spi_transaction_ext_t を使う

        // 簡単に全二重で 13バイト(アドレス1+データ12)やりとりする例
        uint8_t tx_buf[13] = {0};
        // 読みレジスタ(上位ビットセット)
        tx_buf[0] = (ICM42688_REG_ACCEL_GYRO | 0x80);

        // 全二重通信を使い、tx_buf[0]を送信しつつrx_bufに受信する
        // ただし rx_buf[0] にはアドレス送信時のゴミが入るため
        // データは rx_buf[1] から12バイト分になる
        _spi->transmit(tx_buf, rx_buf, 13, _handle);

        // rx_buf[1..12] = 12バイトがセンサデータ
        out[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
        out[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
        out[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);
        out[3] = (int16_t)((rx_buf[7] << 8) | rx_buf[8]);
        out[4] = (int16_t)((rx_buf[9] << 8) | rx_buf[10]);
        out[5] = (int16_t)((rx_buf[11] << 8) | rx_buf[12]);
    }

private:
    static constexpr const char *TAG = "ICM42688";

    SPICreate *_spi;
    spi_device_handle_t _handle;
};
