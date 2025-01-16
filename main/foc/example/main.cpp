/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "foc/esp_foc.h"
#include "svpwm/esp_svpwm.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "SPICREATE.h"
#include "ICM42688.h"

extern "C" void app_main(void)
{
        // SPIラッパークラスインスタンス
    static SPICreate spi;
    // SPI2_HOSTを使う例 (ESP32なら VSPI_HOST=SPI3_HOST, HSPI_HOST=SPI2_HOST など)
    bool ret = spi.begin(
        SPI2_HOST,
        (gpio_num_t)6,   // SCLK
        (gpio_num_t)4,   // MISO
        (gpio_num_t)5,   // MOSI
        8 * 1000 * 1000 // 8MHz
    );
    if (ret != true) {
        printf("SPI begin failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // ICM42688インスタンス
    ICM icm;
    // CS=GPIO_NUM_9(例)、周波数=8MHz
    icm.begin(&spi, (gpio_num_t)40, 8 * 1000 * 1000);
}