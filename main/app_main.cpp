#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "foc/esp_foc.h"
#include "svpwm/esp_svpwm.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "SPICREATE.h"
#include "ICM42688.h"
#include "gptimer.h"
#include "SimpleQuat.h"

extern "C" void app_main(void)
{
}