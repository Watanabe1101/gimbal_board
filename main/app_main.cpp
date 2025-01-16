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

//1Mhz分解能 -> alarm_count == 1000
#define TIMER_RESOLUTION_HZ (1000*1000) //1Mhz
#define TIMER_ALARM_COUNT (1000) //1000ticks = 1ms

typedef struct {
    int16_t sensor[6];
} sensor_data_t;

static QueueHandle_t g_sensorQueue = nullptr;

static ICM icm;

static bool IRAM_ATTR sensorTimerCallback(gptimer_handle_t timer,
                                          const gptimer_alarm_event_data_t *edata,
                                          void *user_ctx)
{
    sensor_data_t sdata;
    
    int16_t sensor[6] = {0};
    icm.Get(sensor);
    for(int i=0; i<6; i++){
        sdata.sensor[i] = sensor[i];
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_sensorQueue, &sdata, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken == pdTRUE);
}

//スケーリング係数
static constexpr float GYRO_SCALE_2000DPS =
    (1.0f / 16.4f) * (3.1415926535f / 180.0f);

extern "C" void app_main(void)
{
    // 1. SPIバス初期化
    static SPICreate spi;
    bool ret = spi.begin(
        SPI2_HOST,
        (gpio_num_t)6,   // SCLK
        (gpio_num_t)4,   // MISO
        (gpio_num_t)5,   // MOSI
        8 * 1000 * 1000  // 8MHz
    );
    if (!ret) {
        printf("SPI begin failed\n");
        return;
    }

    // 2. ICM42688初期化
    //    CS=GPIO_NUM_40 (例)、周波数=8MHz
    icm.begin(&spi, (gpio_num_t)40, 8 * 1000 * 1000);

    // 3. センサデータを受け取るキューを作成
    g_sensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
    if (!g_sensorQueue) {
        printf("Failed to create sensor queue\n");
        return;
    }

    // 4. GPTimerインスタンスを生成＆初期化 (1MHz, alarm=1000)
    static GPTimer gpt;
    if (!gpt.init(TIMER_RESOLUTION_HZ, TIMER_ALARM_COUNT)) {
        printf("Failed to init GPTimer\n");
        return;
    }

    // 5. コールバック登録
    if (!gpt.registerCallback(sensorTimerCallback)) {
        printf("Failed to register GPTimer callback\n");
        return;
    }

    // 6. タイマー開始
    if (!gpt.start()) {
        printf("Failed to start GPTimer\n");
        return;
    }

    static SimpleQuat quat(GYRO_SCALE_2000DPS, 0.001f);

    // 7. メインループ: 1kHzでISRから送られるセンサデータを表示
    while (true) {
        sensor_data_t recvData;
        if(xQueueReceive(g_sensorQueue, &recvData, portMAX_DELAY) == pdTRUE){
            // 角速度は recvData.sensor[3..5] と仮定
            quat.updateFromRawGyro(&recvData.sensor[3]);

            // オイラー角(RAD)を取得
            float euler[3];
            quat.getEulerRad(euler);

            // 必要に応じて deg に変換して表示
            float roll_deg  = euler[0] * (180.0f/M_PI);
            float pitch_deg = euler[1] * (180.0f/M_PI);
            float yaw_deg   = euler[2] * (180.0f/M_PI);

            printf("Roll=%.4f deg, Pitch=%.4f deg, Yaw=%.4f deg\n",
                   roll_deg, pitch_deg, yaw_deg);
        }
    }
}