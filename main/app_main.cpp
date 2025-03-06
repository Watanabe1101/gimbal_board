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
#include "ICM42688.hpp" // ユーザ定義IMUライブラリ
#include "gptimer.hpp"
#include "SimpleQuat.hpp"

// SDMMC Logger (バッファリング対応版を使用してください)
#include "SdmmcLogger.hpp"

// rollモーター
#include "esp_simplefoc.h"

/// @brief ピッチモーターの設定
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(38, 45, 48);
AS5048a sensor1(SPI3_HOST, (gpio_num_t)6, (gpio_num_t)4, (gpio_num_t)15, (gpio_num_t)39);

/// @brief ロールモーターの設定
static BLDCMotor motor2 = BLDCMotor(7); // 同様に7極ペア
static BLDCDriver3PWM driver2 = BLDCDriver3PWM(10, 11, 12);
AS5048a sensor2(SPI3_HOST, (gpio_num_t)6, (gpio_num_t)4, (gpio_num_t)15, (gpio_num_t)7);

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
static QueueHandle_t g_angleQueue = nullptr;  // 角度指示用キュー
static SPICreate g_spi;                       // SPIクラス
static ICM g_icm;                             // IMU
static SdmmcLogger g_sdCard;                  // SDカードロガー
static GPTimer g_gpt;                         // GPTimer

// ------------------ 目標角(roll, pitch)共有用 ------------------
static float roll_target = 0.0f;
static float pitch_target = 0.0f;
static SemaphoreHandle_t g_angleMutex = nullptr;
// --------------------------------------------------------------

// 角速度スケーリング係数 (ICM42688 ±2000dps相当)
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
        (gpio_num_t)8,  // SCLK
        (gpio_num_t)16, // MISO
        (gpio_num_t)5,  // MOSI
        8 * 1000 * 1000 // 8MHz
    );
    if (!ret)
    {
        ESP_LOGE("sensorTask", "SPI begin failed");
        vTaskDelete(NULL);
        return;
    }

    // 2. ICM42688初期化
    g_icm.begin(&g_spi, (gpio_num_t)40, 8 * 1000 * 1000);

    // 3. キューを作成 (深さは必要に応じて調整)
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
    // コールバック登録 (ISRでセンサー読み込み & キュー送信)
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

    // 割り込みが動いている間、ここでは特にすることがない
    // 必要に応じて他の処理を入れてもよい
    while (true)
    {
        // 1秒に1回程度ログを出すなど
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
    // 1. SDカード初期化 (バッファリング対応)
    //    例: HighSpeed = false, mountPoint="/sdcard", logFile="/sdcard/gyro_log.csv"
    bool useHighSpeed = false;
    if (!g_sdCard.begin(useHighSpeed, "/sdcard", "/sdcard/gyro_log.csv"))
    {
        ESP_LOGE("loggingTask", "Failed to mount or open log file.");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI("loggingTask", "SD card initialized");

    // 2. クォータニオン演算用インスタンス (サンプリング周期=1kHz → 0.001f)
    SimpleQuat quat(GYRO_SCALE_2000DPS, 0.001f);

    // -----------------------------
    // 2-1. キャリブレーション
    // -----------------------------
    ESP_LOGI("loggingTask", "Start gyro bias calibration for 60sec...");
    const int sample_count = 1000 * 6; // 60秒
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int countSample = 0;

    for (int i = 0; i < sample_count; i++)
    {
        sensor_data_t data;
        // キューからデータを取り出す (60秒間は連続で待つ)
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
    uint32_t printcount = 0; // コンソール出力用カウンタ
    while (true)
    {
        sensor_data_t recvData;
        // センサデータを待つ
        if (xQueueReceive(g_sensorQueue, &recvData, portMAX_DELAY) == pdTRUE)
        {
            // クォータニオン更新(角速度=recvData.sensor[3..5])
            quat.updateFromRawGyro(&recvData.sensor[3]);

            // オイラー角計算(ラジアン)
            float euler[3];
            quat.getEulerRad(euler);

            float target_angle = euler[0];
            // // モーター指示
            // if (g_angleQueue != nullptr)
            // {
            //     xQueueOverwrite(g_angleQueue, &target_angle);
            // }
            // 度数法に変換
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

            // ログファイルへ書き込み
            g_sdCard.writeLog(
                recvData.timestamp_us,
                recvData.sensor[3],
                recvData.sensor[4],
                recvData.sensor[5],
                roll_deg, pitch_deg, yaw_deg);

            // 100回に1回コンソールへ
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

    // (実際にタスクが終了する場合はファイルをクローズ)
    // g_sdCard.end();
    vTaskDelete(NULL);
}

// -----------------------------
// 角度制御タスク (モータ2ch)
// -----------------------------
#define USING_MCPWM
static void angleControlTask(void *args)
{
    // 1. ドライバ初期化 & モータ設定
    sensor1.init();
    motor1.linkSensor(&sensor1);
    driver1.voltage_power_supply = 7;
    driver1.voltage_limit = 4;
    driver1.init(0); // MCPWMユニット0
    motor1.linkDriver(&driver1);
    motor1.controller = MotionControlType::angle;
    motor1.velocity_limit = 100.0; // [rad/s]上限
    motor1.voltage_limit = 4.0;
    motor1.init();
    motor1.initFOC();

    driver2.voltage_power_supply = 7;
    driver2.voltage_limit = 4;
    driver2.init(1); // MCPWMユニット1
    sensor2.init();
    motor2.linkSensor(&sensor2);
    motor2.linkDriver(&driver2);
    motor2.controller = MotionControlType::angle;
    motor2.velocity_limit = 100.0;
    motor2.voltage_limit = 6.0;
    motor2.init();
    motor2.initFOC();

    ESP_LOGI("angleControlTask", "Motors initialized (open-loop angle).");

    // 角度更新のダウンサンプリング
    int16_t downsample_counter = 10;
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
        float target_angle_roll = local_roll * 3.1415926535f / 180.0f;
        float target_angle_pitch = local_pitch * 3.1415926535f / 180.0f;

        // モータ1,2 それぞれ角度指令を与える（オープンループ）
        motor1.loopFOC();
        motor2.loopFOC();
        if (downsample_counter-- <= 0)
        {
            motor1.move(target_angle_roll);
            motor2.move(target_angle_pitch);
            downsample_counter = 10;
        }
        else
        {
            motor1.move();
            motor2.move();
        }

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

    // センサータスク作成
    xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, NULL, 5, NULL, 1);
    // ロギングタスク作成
    xTaskCreatePinnedToCore(loggingTask, "loggingTask", 4096, NULL, 5, NULL, 1);

    xTaskCreatePinnedToCore(angleControlTask, "angleControlTask", 8192, NULL, 6, NULL, 0);
    // メインタスクは何もしない場合
    // (不要なら vTaskDelete(NULL) で自滅しても可)
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}