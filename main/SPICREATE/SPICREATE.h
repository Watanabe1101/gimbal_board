#pragma once

#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CSアサート(CS=LOW)用コールバック関数
 *        - spi_device_interface_config_t の pre_cb に指定
 */
void IRAM_ATTR csReset(spi_transaction_t *t);

/**
 * @brief CSデアサート(CS=HIGH)用コールバック関数
 *        - spi_device_interface_config_t の post_cb に指定
 */
void IRAM_ATTR csSet(spi_transaction_t *t);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief SPIバスをラップするクラス
 */
class SPICreate
{
public:
    /**
     * @brief コンストラクタ
     */
    SPICreate();

    /**
     * @brief デストラクタ
     */
    ~SPICreate();

    /**
     * @brief SPIバスの初期化
     * @param host      SPIホスト (SPI2_HOST, SPI3_HOST など)
     * @param sck       SCLKピン番号
     * @param miso      MISOピン番号
     * @param mosi      MOSIピン番号
     * @param freq      SPIクロック周波数(Hz)
     * @param dma_chan  DMAチャネル (0=DMAなし, 1 or 2 or SPI_DMA_CH_AUTOなど)
     * @return ESP_OKなら成功、それ以外は失敗
     */
    esp_err_t begin(spi_host_device_t host,
                    gpio_num_t sck,
                    gpio_num_t miso,
                    gpio_num_t mosi,
                    uint32_t freq,
                    int dma_chan = SPI_DMA_CH_AUTO);

    /**
     * @brief SPIバスの解放
     * @return ESP_OKなら成功
     */
    esp_err_t end();

    /**
     * @brief SPIデバイスを追加
     * @param if_cfg spi_device_interface_config_t 構造体へのポインタ
     * @param csPin  CSピン番号 (ソフト制御や pre_cb/post_cb 内で利用)
     * @return 正常に追加された場合はデバイスハンドル、失敗時にはnullptrを返す
     */
    spi_device_handle_t addDevice(const spi_device_interface_config_t *if_cfg,
                                  gpio_num_t csPin);

    /**
     * @brief SPIデバイスを削除
     * @param handle addDevice() で返却されたデバイスハンドル
     * @return ESP_OKなら成功
     */
    esp_err_t removeDevice(spi_device_handle_t handle);

    /**
     * @brief 1バイトコマンド送信
     * @param cmd    送信コマンド
     * @param handle デバイスハンドル
     */
    void sendCmd(uint8_t cmd, spi_device_handle_t handle);

    /**
     * @brief 1バイトレジスタ読み
     * @param addr   読み出しレジスタアドレス(デバイスによってフォーマットが異なる)
     * @param handle デバイスハンドル
     * @return 読み出し結果(1バイト)
     */
    uint8_t readByte(uint8_t addr, spi_device_handle_t handle);

    /**
     * @brief 1バイトレジスタ書き込み
     * @param addr   書き込み先レジスタアドレス
     * @param data   書き込む値
     * @param handle デバイスハンドル
     */
    void writeByte(uint8_t addr, uint8_t data, spi_device_handle_t handle);

    /**
     * @brief 任意バイト数の全二重送受信 (tx/rx)
     */
    void transmit(const uint8_t *tx, uint8_t *rx, size_t len, spi_device_handle_t handle);

    /**
     * @brief spi_transaction_t を使った送受信
     *        - 内部で spi_device_transmit() を呼ぶ (ブロッキング)
     */
    void transmit(spi_transaction_t *trans, spi_device_handle_t handle);

    /**
     * @brief spi_transaction_t を使ったポーリング送受信
     *        - 内部で spi_device_polling_transmit() を呼ぶ (ブロッキング)
     */
    void pollTransmit(spi_transaction_t *trans, spi_device_handle_t handle);

private:
    static const char *TAG;

    spi_host_device_t _host;
    bool _initialized;
};

#endif // __cplusplus