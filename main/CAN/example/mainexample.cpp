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

// CAN通信
#include "CanComm.hpp"

#define TIMER_RESOLUTION_HZ (1000 * 1000) // 1MHz
#define TIMER_ALARM_COUNT (1000)          // 1000ticks = 1ms(=1kHz)

// センサデータをキューでやりとりするための構造体
typedef struct
{
    int16_t sensor[6];
    uint64_t timestamp_us;
} sensor_data_t;

// CAN通信の初期化
CanComm CAN(BoardID::GIMBAL, GPIO_NUM_14, GPIO_NUM_13);

// グローバル変数
static QueueHandle_t g_sensorQueue = nullptr; // センサー生データ用キュー
static SPICreate g_spi;                       // SPIクラス
static ICM g_icm;                             // IMU
static SdmmcLogger g_sdCard;                  // SDカードロガー
static GPTimer g_gpt;                         // GPTimer

// 角速度スケーリング係数 (ICM42688 ±2000dps相当)
static constexpr float GYRO_SCALE_2000DPS = (1.0f / 16.4f) * (3.1415926535f / 180.0f);

// -------------------------
// モード管理用の定義
// -------------------------
enum class OperationMode
{
    START,       // 初期化時のモード。CANのみ動作、その他タスクは未起動／停止状態
    PREPARATION, // センサ・ロギングタスクを動作
    LOGGING      // 今後追加予定のタスクを動作（現状はPREPARATIONタスクはそのまま動作）
};

// グローバルモード変数（初期状態はSTART）
static volatile OperationMode g_currentMode = OperationMode::START;

// 各タスクのタスクハンドル（タスク生成時にセット）
static TaskHandle_t sensorTaskHandle = nullptr;
static TaskHandle_t loggingTaskHandle = nullptr;
static TaskHandle_t canCommandTaskHandle = nullptr;

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
    const int sample_count = 1000 * 5; // 5秒
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

    int16_t ScaledQuarternion[4] = {0};
    uint8_t PackScaledQuarternion[8] = {0};
    float Quartrenion[4] = {0};

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

            // 度数法に変換
            float roll_deg = euler[0] * (180.0f / 3.1415926535f);
            float pitch_deg = euler[1] * (180.0f / 3.1415926535f);
            float yaw_deg = euler[2] * (180.0f / 3.1415926535f);

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
                g_sdCard.flush(); // sd flush
                quat.GetScaledQuarternion(ScaledQuarternion);
                quat.GetQuarternion(Quartrenion);
                // ESP_LOGI("loggingTask", "[%.2f ms] GYRO=(%d,%d,%d), EULER=(%.2f, %.2f, %.2f)",
                //          recvData.timestamp_us / 1000.0,
                //          recvData.sensor[3], recvData.sensor[4], recvData.sensor[5],
                //          roll_deg, pitch_deg, yaw_deg);
                // can送信用にデータをパック
                for (int i = 0; i < 4; i++)
                {
                    PackScaledQuarternion[i * 2] = (uint8_t)(ScaledQuarternion[i] & 0x00FF);
                    PackScaledQuarternion[i * 2 + 1] = (uint8_t)((ScaledQuarternion[i] & 0xFF00) >> 8);
                }
                CAN.send(ContentID::QUATERNION, PackScaledQuarternion, 8);
                ESP_LOGI("loggingTask", "[%.2f ms] Quarternion=(%d,%d,%d,%d), Quarternion=(%.2f, %.2f, %.2f, %.2f)",
                         recvData.timestamp_us / 1000.0,
                         ScaledQuarternion[0], ScaledQuarternion[1], ScaledQuarternion[2], ScaledQuarternion[3],
                         Quartrenion[0], Quartrenion[1], Quartrenion[2], Quartrenion[3]);

                printcount = 0;
            }
        }
    }

    // (実際にタスクが終了する場合はファイルをクローズ)
    // g_sdCard.end();
    vTaskDelete(NULL);
}

// -------------------------
// CANコマンド受信タスク
// -------------------------
// このタスクはCANバスからのモード遷移コマンドを受信し、
// g_currentMode を更新するとともに、対応するタスクの起動／停止を行います。
static void canCommandTask(void *args)
{
    ESP_LOGI("canCommandTask", "Starting CAN command task...");
    CanRxFrame rxFrame;

    while (true)
    {
        // 非ブロッキングでCANフレームを取得
        if (CAN.readFrameNoWait(rxFrame) == ESP_OK)
        {
            if (rxFrame.content_id == ContentID::MODE_TRANSITION)
            {
                // 受信データの先頭1バイトにコマンド文字が入っている前提
                char cmd = (char)rxFrame.data[0];
                ESP_LOGI("canCommandTask", "Received mode command: %c", cmd);

                // コマンドによりモード遷移
                if (cmd == (char)ModeCommand::PREPARATION)
                { // 'p'
                    if (g_currentMode == OperationMode::START)
                    {
                        g_currentMode = OperationMode::PREPARATION;
                        // センサタスクとロギングタスクの生成
                        if (sensorTaskHandle == nullptr)
                        {
                            xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, NULL, 5, &sensorTaskHandle, 1);
                        }
                        if (loggingTaskHandle == nullptr)
                        {
                            xTaskCreatePinnedToCore(loggingTask, "loggingTask", 4096, NULL, 5, &loggingTaskHandle, 1);
                        }
                        ESP_LOGI("canCommandTask", "Mode transition: START -> PREPARATION");
                    }
                }
                else if (cmd == (char)ModeCommand::LOGGING)
                { // 'l'
                    if (g_currentMode == OperationMode::PREPARATION)
                    {
                        g_currentMode = OperationMode::LOGGING;
                        // ※必要に応じて、logging mode専用タスクの起動処理などを追加
                        ESP_LOGI("canCommandTask", "Mode transition: PREPARATION -> LOGGING");
                    }
                }
                else if (cmd == (char)ModeCommand::START)
                { // 's'
                    if (g_currentMode == OperationMode::PREPARATION || g_currentMode == OperationMode::LOGGING)
                    {
                        g_currentMode = OperationMode::START;
                        // 実行中のセンサタスク・ロギングタスクを停止（タスク削除）
                        if (sensorTaskHandle != nullptr)
                        {
                            vTaskDelete(sensorTaskHandle);
                            sensorTaskHandle = nullptr;
                        }
                        if (loggingTaskHandle != nullptr)
                        {
                            vTaskDelete(loggingTaskHandle);
                            loggingTaskHandle = nullptr;
                        }
                        ESP_LOGI("canCommandTask", "Mode transition: PREPARATION/LOGGING -> START");
                    }
                }
                else if (cmd == 'b')
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    CAN.send(ContentID::BOARD_STATE, reinterpret_cast<const uint8_t *>("b"), 1);
                }

                else
                {
                    ESP_LOGW("canCommandTask", "Unknown mode command received: %c", cmd);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 少し待ってループ（busy loop回避）
    }
    vTaskDelete(NULL);
}

// -----------------------------
// メイン (タスク生成)
// -----------------------------
// -------------------------
// メイン (タスク生成)
// -------------------------
extern "C" void app_main(void)
{
    ESP_LOGI("app_main", "System starting in START mode...");
    // CANドライバ初期化
    if (CAN.begin() != ESP_OK)
    {
        ESP_LOGE("app_main", "CAN begin failed");
        return;
    }

    // CANコマンド受信用タスクを常時動作させる
    xTaskCreatePinnedToCore(canCommandTask, "canCommandTask", 4096, NULL, 6, &canCommandTaskHandle, 1);

    // ※初期モードはSTARTのため、センサ・ロギングタスクは生成せず、
    //     CANからのモード遷移コマンドを待つ。
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGD("app_main", "Main loop in START mode...");
    }
}