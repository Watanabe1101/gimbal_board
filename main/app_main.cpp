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

// static const char *TAG = "example_foc";
// static const char *TAG_IMU = "IMU";

// //////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////// Please update the following configuration according to your HardWare spec /////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////
// #define EXAMPLE_FOC_DRV_EN_GPIO          46
// #define EXAMPLE_FOC_DRV_FAULT_GPIO       9
// #define EXAMPLE_FOC_PWM_UH_GPIO          10
// #define EXAMPLE_FOC_PWM_UL_GPIO          21
// #define EXAMPLE_FOC_PWM_VH_GPIO          11
// #define EXAMPLE_FOC_PWM_VL_GPIO          13
// #define EXAMPLE_FOC_PWM_WH_GPIO          12
// #define EXAMPLE_FOC_PWM_WL_GPIO          14

// #define EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
// #define EXAMPLE_FOC_MCPWM_PERIOD              400     // 1000 * 0.1us = 100us, 10KHz

// #define EXAMPLE_FOC_WAVE_FREQ    1         // 50Hz 3 phase AC wave
// #define EXAMPLE_FOC_WAVE_AMPL    100        // Wave amplitude, Use up-down timer mode, max value should be (EXAMPLE_FOC_MCPWM_PERIOD/2)

// void bsp_bridge_driver_init(void)
// {
//     gpio_config_t drv_en_config = {
//         .mode = GPIO_MODE_OUTPUT,
//         .pin_bit_mask = 1ULL << EXAMPLE_FOC_DRV_EN_GPIO,
//     };
//     ESP_ERROR_CHECK(gpio_config(&drv_en_config));
// }

// void bsp_bridge_driver_enable(bool enable)
// {
//     ESP_LOGI(TAG, "%s MOSFET gate", enable ? "Enable" : "Disable");
//     gpio_set_level(EXAMPLE_FOC_DRV_EN_GPIO, enable);
// }

// bool IRAM_ATTR inverter_update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
// {
//     BaseType_t task_yield = pdFALSE;
//     xSemaphoreGiveFromISR(*((SemaphoreHandle_t *)user_ctx), &task_yield);
//     return task_yield;
// }

// void app_main(void)
// {
//     esp_log_level_set("*", ESP_LOG_INFO);
    // ESP_LOGI(TAG, "Hello FOC");
    // // counting semaphore used to sync update foc calculation when mcpwm timer updated
    // SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);

    // foc_dq_coord_t dq_out = {_IQ(0), _IQ(0)};
    // foc_ab_coord_t ab_out;
    // foc_uvw_coord_t uvw_out;
    // int uvw_duty[3];
    // float elec_theta_deg = 0;
    // _iq elec_theta_rad;

    // inverter_config_t cfg = {
    //     .timer_config = {
    //         .group_id = 0,
    //         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    //         .resolution_hz = EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ,
    //         .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,   //UP_DOWN mode will generate center align pwm wave, which can reduce MOSFET switch times on same effect, extend life
    //         .period_ticks = EXAMPLE_FOC_MCPWM_PERIOD,
    //     },
    //     .operator_config = {
    //         .group_id = 0,
    //     },
    //     .compare_config = {
    //         .flags.update_cmp_on_tez = true,
    //     },
    //     .gen_gpios = {
    //         {EXAMPLE_FOC_PWM_UH_GPIO, EXAMPLE_FOC_PWM_UL_GPIO},
    //         {EXAMPLE_FOC_PWM_VH_GPIO, EXAMPLE_FOC_PWM_VL_GPIO},
    //         {EXAMPLE_FOC_PWM_WH_GPIO, EXAMPLE_FOC_PWM_WL_GPIO},
    //     },
    //     .dt_config = {
    //         .posedge_delay_ticks = 5,
    //     },
    //     .inv_dt_config = {
    //         .negedge_delay_ticks = 5,
    //         .flags.invert_output = true,
    //     },
    // };
    // inverter_handle_t inverter1;
    // ESP_ERROR_CHECK(svpwm_new_inverter(&cfg, &inverter1));
    // ESP_LOGI(TAG, "Inverter init OK");

    // mcpwm_timer_event_callbacks_t cbs = {
    //     .on_full = inverter_update_cb,
    // };
    // ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter1, &cbs, &update_semaphore));
    // ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_START_NO_STOP));
    // ESP_LOGI(TAG, "Inverter start OK");

    // // Enable gate driver chip
    // bsp_bridge_driver_init();
    // bsp_bridge_driver_enable(true);

    // ESP_LOGI(TAG, "Start FOC");

    // int foc_iteration_count = 0; // カウンタ初期化

    // while (true) {
    //     xSemaphoreTake(update_semaphore, portMAX_DELAY);

    //     // 処理時間計測の開始
    //     int64_t start_time = esp_timer_get_time();

    //     // Calculate elec_theta_deg increase step of 50Hz output on 10000Hz call
    //     elec_theta_deg += (EXAMPLE_FOC_WAVE_AMPL * 360.f) / (EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ / EXAMPLE_FOC_WAVE_FREQ);
    //     if (elec_theta_deg > 360) {
    //         elec_theta_deg -= 360;
    //     }
    //     elec_theta_rad = _IQmpy(_IQ(elec_theta_deg), _IQ(M_PI / 180.f));

    //     // In FOC motor control, we usually set Vd for alignment or weak-meg control, and set Vq for torque control.
    //     // As here is open loop output, use Vd is enough, and coord aligned
    //     dq_out.d = _IQ(EXAMPLE_FOC_WAVE_AMPL);
    //     foc_inverse_park_transform(elec_theta_rad, &dq_out, &ab_out);

    // #if CONFIG_ESP_FOC_USE_SVPWM
    //     foc_svpwm_duty_calculate(&ab_out, &uvw_out);
    // #else   // Use spwm (sin pwm) instead. (see menuconfig help to know difference between SVPWM and SPWM)
    //     foc_inverse_clarke_transform(&ab_out, &uvw_out);
    // #endif
    //     // Regular uvw data to (0 ~ (EXAMPLE_FOC_MCPWM_PERIOD/2))
    //     uvw_duty[0] = _IQtoF(_IQdiv2(uvw_out.u)) + (EXAMPLE_FOC_MCPWM_PERIOD / 4);
    //     uvw_duty[1] = _IQtoF(_IQdiv2(uvw_out.v)) + (EXAMPLE_FOC_MCPWM_PERIOD / 4);
    //     uvw_duty[2] = _IQtoF(_IQdiv2(uvw_out.w)) + (EXAMPLE_FOC_MCPWM_PERIOD / 4);

    //     // output pwm duty
    //     ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter1, uvw_duty[0], uvw_duty[1], uvw_duty[2]));

    //     // 処理時間計測の終了
    //     int64_t end_time = esp_timer_get_time();
    //     foc_iteration_count++;

    //     // 100回に1回処理時間を出力
    //     if (foc_iteration_count % 100 == 0) {
    //         ESP_LOGI(TAG, "FOC iteration %d: Processing time = %lld us", foc_iteration_count, end_time - start_time);
    //     }
    // }

    // bsp_bridge_driver_enable(false);
    // ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_STOP_EMPTY));
    // ESP_ERROR_CHECK(svpwm_del_inverter(inverter1));
// }

extern "C" void app_main(void)
{
    // SPIラッパークラスインスタンス
    static SPICreate spi;
    // SPI2_HOSTを使う例 (ESP32なら VSPI_HOST=SPI3_HOST, HSPI_HOST=SPI2_HOST など)
    esp_err_t ret = spi.begin(
        SPI2_HOST,
        (gpio_num_t)6,   // SCLK
        (gpio_num_t)4,   // MISO
        (gpio_num_t)5,   // MOSI
        8 * 1000 * 1000 // 8MHz
    );
    if (ret != ESP_OK) {
        printf("SPI begin failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // ICM42688インスタンス
    ICM icm;
    // CS=GPIO_NUM_9(例)、周波数=8MHz
    icm.begin(&spi, (gpio_num_t)40, 8 * 1000 * 1000);

    while (true) {
        uint8_t id = icm.WhoAmI();
        printf("WHO_AM_I=0x%02X\n", id);

        int16_t sensor[6] ={0};
        icm.Get(sensor);
        printf("Accel: [%d, %d, %d], Gyro: [%d, %d, %d]\n",
               sensor[0], sensor[1], sensor[2],
               sensor[3], sensor[4], sensor[5]);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
