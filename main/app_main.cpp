#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_system.h"

// SPI & IMU
#include "SPICREATE.hpp"
#include "ICM42688.hpp"
#include "gptimer.hpp"
#include "SimpleQuat.hpp"

// SDMMC Logger
#include "SdmmcLogger.hpp"

// ------------------ ここから SimpleFOC 追加 ------------------
#include "esp_simplefoc.h"

// [例] 2つのBLDCモータを定義（極数やPWMピンは実際の配線に応じて設定してください）
// モータ1
static BLDCMotor motor1 = BLDCMotor(7); // 極数=7極ペア (14極モータ) の例
static BLDCDriver3PWM driver1 = BLDCDriver3PWM(38, 45, 48);

// モータ2
static BLDCMotor motor2 = BLDCMotor(7); // 同様に7極ペア
static BLDCDriver3PWM driver2 = BLDCDriver3PWM(10, 11, 12);
// ------------------------------------------------------------

#define TIMER_RESOLUTION_HZ (1000 * 1000) // 1MHz
#define TIMER_ALARM_COUNT (1000)          // 1000ticks = 1ms(=1kHz)

// センサデータをキューでやりとりするための構造体
typedef struct
{
    int16_t sensor[6];
    uint64_t timestamp_us;
} sensor_data_t;

// グローバル変数
static QueueHandle_t g_sensorQueue = nullptr; // センサー生データ用キュー
static SPICreate g_spi;                       // SPIクラス
static ICM g_icm;                             // IMU
static SdmmcLogger g_sdCard;                  // SDカードロガー
static GPTimer g_gpt;                         // GPTimer

// ------------------ 目標角(roll, pitch)共有用 ------------------
static float roll_target = 0.0f;
static float pitch_target = 0.0f;
static SemaphoreHandle_t g_angleMutex = nullptr;
// --------------------------------------------------------------

// 角速度スケーリング係数 (ICM42688 ±2000dps想定)
static constexpr float GYRO_SCALE_2000DPS = (1.0f / 16.4f) * (3.1415926535f / 180.0f);

// 割り込みコールバック(1kHz)
static bool IRAM_ATTR sensorTimerCallback(gptimer_handle_t timer,
                                          const gptimer_alarm_event_data_t *edata,
                                          void *user_ctx)
{
    // IMUからセンサ値を取得
    sensor_data_t sdata;
    int16_t sensor[6] = {0};
    g_icm.Get(sensor);

    for (int i = 0; i < 6; i++)
    {
        sdata.sensor[i] = sensor[i];
    }
    // タイムスタンプ(μs)
    sdata.timestamp_us = (uint64_t)esp_timer_get_time();

    // キューへ送る
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_sensorQueue, &sdata, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken == pdTRUE);
}

// -----------------------------
// センサー側タスク
// -----------------------------
static void sensorTask(void *args)
{
    // 1. SPI初期化
    bool ret = g_spi.begin(
        SPI2_HOST,
        (gpio_num_t)6,  // SCLK
        (gpio_num_t)4,  // MISO
        (gpio_num_t)5,  // MOSI
        1 * 1000 * 1000 // 8MHz
    );
    if (!ret)
    {
        ESP_LOGE("sensorTask", "SPI begin failed");
        vTaskDelete(NULL);
        return;
    }

    // 2. ICM42688初期化
    g_icm.begin(&g_spi, (gpio_num_t)40, 8 * 1000 * 1000);

    // 3. キュー作成
    g_sensorQueue = xQueueCreate(512, sizeof(sensor_data_t));
    if (!g_sensorQueue)
    {
        ESP_LOGE("sensorTask", "Failed to create sensor queue");
        vTaskDelete(NULL);
        return;
    }

    // 4. GPTimer初期化 (1MHz, アラーム=1000 → 1kHz)
    if (!g_gpt.init(TIMER_RESOLUTION_HZ, TIMER_ALARM_COUNT))
    {
        ESP_LOGE("sensorTask", "Failed to init GPTimer");
        vTaskDelete(NULL);
        return;
    }
    if (!g_gpt.registerCallback(sensorTimerCallback))
    {
        ESP_LOGE("sensorTask", "Failed to register GPTimer callback");
        vTaskDelete(NULL);
        return;
    }

    // 5. タイマー開始
    if (!g_gpt.start())
    {
        ESP_LOGE("sensorTask", "Failed to start GPTimer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("sensorTask", "Sensor reading started (1kHz by GPTimer)");

    // タスクループ
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGD("sensorTask", "Sensor task alive...");
    }

    vTaskDelete(NULL);
}

// -----------------------------
// ロギングタスク
// -----------------------------
static void loggingTask(void *args)
{
    // 1. SDカード初期化
    bool useHighSpeed = false;
    if (!g_sdCard.begin(useHighSpeed, "/sdcard", "/sdcard/gyro_log.csv"))
    {
        ESP_LOGE("loggingTask", "Failed to mount or open log file.");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI("loggingTask", "SD card initialized");

    // 2. クォータニオン (サンプリング周期=1kHz → 0.001f)
    SimpleQuat quat(GYRO_SCALE_2000DPS, 0.001f);

    // -----------------------------
    // 2-1. ジャイロバイアス補正 (60秒)
    // -----------------------------
    ESP_LOGI("loggingTask", "Start gyro bias calibration for 60sec...");
    const int sample_count = 1000 * 5;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int countSample = 0;

    for (int i = 0; i < sample_count; i++)
    {
        sensor_data_t data;
        if (xQueueReceive(g_sensorQueue, &data, portMAX_DELAY) == pdTRUE)
        {
            sumX += data.sensor[3];
            sumY += data.sensor[4];
            sumZ += data.sensor[5];
            countSample++;
        }
    }

    float offsetX = (countSample > 0) ? (sumX / countSample) : 0.0f;
    float offsetY = (countSample > 0) ? (sumY / countSample) : 0.0f;
    float offsetZ = (countSample > 0) ? (sumZ / countSample) : 0.0f;
    quat.setGyroBias(offsetX, offsetY, offsetZ);

    ESP_LOGI("loggingTask", "Calibration done! Gyro bias = (%.2f, %.2f, %.2f)",
             offsetX, offsetY, offsetZ);

    // -----------------------------
    // 2-2. 通常ロギング
    // -----------------------------
    uint32_t printcount = 0;
    while (true)
    {
        sensor_data_t recvData;
        if (xQueueReceive(g_sensorQueue, &recvData, portMAX_DELAY) == pdTRUE)
        {
            // クォータニオン更新
            quat.updateFromRawGyro(&recvData.sensor[3]);

            // オイラー角(rad) → deg
            float euler[3];
            quat.getEulerRad(euler);
            float roll_deg = euler[0] * (180.0f / 3.1415926535f);
            float pitch_deg = euler[1] * (180.0f / 3.1415926535f);
            float yaw_deg = euler[2] * (180.0f / 3.1415926535f);

            // -----------------------------
            // ロール・ピッチを「目標角」として共有
            // -----------------------------
            if (g_angleMutex && xSemaphoreTake(g_angleMutex, 0) == pdTRUE)
            {
                roll_target = yaw_deg;
                pitch_target = pitch_deg;
                xSemaphoreGive(g_angleMutex);
            }

            // ログ書き込み
            g_sdCard.writeLog(
                recvData.timestamp_us,
                recvData.sensor[3],
                recvData.sensor[4],
                recvData.sensor[5],
                roll_deg, pitch_deg, yaw_deg);

            // 100回に1回コンソールへ出力
            if (printcount++ == 100)
            {
                g_sdCard.flush();
                ESP_LOGI("loggingTask", "[%.2f ms] GYRO=(%d,%d,%d), EULER=(%.2f, %.2f, %.2f)",
                         recvData.timestamp_us / 1000.0,
                         recvData.sensor[3], recvData.sensor[4], recvData.sensor[5],
                         roll_deg, pitch_deg, yaw_deg);
                printcount = 0;
            }
        }
    }

    vTaskDelete(NULL);
}

// -----------------------------
// 角度制御タスク (モータ2ch)
// -----------------------------
#define USING_MCPWM
static void angleControlTask(void *args)
{
    // 1. ドライバ初期化 & モータ設定
    driver1.voltage_power_supply = 12;
    driver1.voltage_limit = 6;
#ifdef USING_MCPWM
    driver1.init(0); // MCPWMユニット0

#else
    driver1.init({1, 2, 3});
#endif
    motor1.linkDriver(&driver1);
    motor1.controller = MotionControlType::openloop_angle;
    motor1.velocity_limit = 100.0; // [rad/s]上限
    motor1.voltage_limit = 6.0;
    motor1.init();

    driver2.voltage_power_supply = 12;
    driver2.voltage_limit = 6;
#ifdef USING_MCPWM
    driver2.init(1); // MCPWMユニット1
#else
    driver2.init({4, 5, 6});
#endif
    motor2.linkDriver(&driver2);
    motor2.controller = MotionControlType::openloop_angle;
    motor2.velocity_limit = 100.0;
    motor2.voltage_limit = 6.0;
    motor2.init();

    ESP_LOGI("angleControlTask", "Motors initialized (open-loop angle).");

    while (true)
    {
        float local_roll = 0.0f;
        float local_pitch = 0.0f;

        // ミューテックスで共有データを取得
        if (g_angleMutex && xSemaphoreTake(g_angleMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            local_roll = roll_target;
            local_pitch = pitch_target;
            xSemaphoreGive(g_angleMutex);
        }

        // deg→rad
        float target_angle1 = local_roll * 3.1415926535f / 180.0f;
        float target_angle2 = local_pitch * 3.1415926535f / 180.0f;

        // モータ1,2 それぞれ角度指令を与える（オープンループ）
        motor1.move(target_angle1);
        motor2.move(target_angle2);

        // ループ周期 (約1kHz)
        vTaskDelay(1);
    }

    vTaskDelete(NULL);
}

// -----------------------------
// メイン (タスク生成)
// -----------------------------
extern "C" void app_main(void)
{
    // 目標角共有用ミューテックス
    g_angleMutex = xSemaphoreCreateMutex();

    // 各タスクをコア分離
    xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(loggingTask, "loggingTask", 8192, NULL, 5, NULL, 1);

    // 角度制御タスクは別コアへ
    xTaskCreatePinnedToCore(angleControlTask, "angleControlTask", 8192, NULL, 6, NULL, 0);

    // メインタスクは何もしない
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
