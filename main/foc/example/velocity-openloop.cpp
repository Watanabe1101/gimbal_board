/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_simplefoc.h"

#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif

BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(38, 45, 48);

BLDCMotor motor2 = BLDCMotor(2);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(10, 11, 12);

extern "C" void app_main(void)
{
    SimpleFOCDebug::enable(); /*!< Enable Debug */
    Serial.begin(115200);

    driver.voltage_power_supply = 12;
    driver.voltage_limit = 11;

    driver2.voltage_power_supply = 12;
    driver2.voltage_limit = 11;

#ifdef USING_MCPWM
    driver.init(0);
    driver2.init(1);

#else
    driver.init({1, 2, 3});
    driver2.init({4, 5, 6});
#endif
    motor.linkDriver(&driver);
    motor2.linkDriver(&driver2);

    motor.velocity_limit = 200.0;
    motor.voltage_limit = 12.0;
    motor.controller = MotionControlType::velocity_openloop;

    motor2.velocity_limit = 200.0;
    motor2.voltage_limit = 12.0;
    motor2.controller = MotionControlType::velocity_openloop;

    motor.init();
    motor2.init();

    float speed = 0;
    int count = 0;

    while (1)
    {
        motor.move(speed);
        motor2.move(speed);

        count++;
        if (count > 100)
        {
            speed++;
            count = 0;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
