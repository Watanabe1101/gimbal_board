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
#include "SPICREATE.h"
#include "ICM42688.h" // ユーザ定義IMUライブラリ(質問文で示されたICMクラス)
#include "gptimer.h"
#include "SimpleQuat.h"

// SDMMC Logger
#include "SdmmcLogger.h"

#define TIMER_RESOLUTION_HZ (1000 * 1000) // 1MHz
#define TIMER_ALARM_COUNT (1000)          // 1000ticks = 1ms(=1kHz)

typedef struct
{
    int16_t sensor[6];
    uint64_t timestamp_us; // 計測タイミング
} sensor_data_t;

static QueueHandle_t g_sensorQueue = nullptr;
static ICM icm;            // IMU
static SdmmcLogger sdCard; // SDMMC ロガー

// タイマーISRコールバック (1kHz)
static bool IRAM_ATTR sensorTimerCallback(gptimer_handle_t timer,
                                          const gptimer_alarm_event_data_t *edata,
                                          void *user_ctx)
{
    // IMUからセンサ値を読む
    sensor_data_t sdata;
    int16_t sensor[6] = {0};
    icm.Get(sensor);
    for (int i = 0; i < 6; i++)
    {
        sdata.sensor[i] = sensor[i];
    }

    // タイムスタンプ取得 (μs)
    sdata.timestamp_us = (uint64_t)esp_timer_get_time();

    // キューに書き込む
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_sensorQueue, &sdata, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken == pdTRUE);
}

// 角速度スケーリング係数
static constexpr float GYRO_SCALE_2000DPS =
    (1.0f / 16.4f) * (3.1415926535f / 180.0f);

// ログ表示頻度
static uint32_t printcount = 0;

extern "C" void app_main(void)
{
    // 1. SPIバス初期化
    static SPICreate spi;
    bool ret = spi.begin(
        SPI2_HOST,
        (gpio_num_t)6,  // SCLK
        (gpio_num_t)4,  // MISO
        (gpio_num_t)5,  // MOSI
        8 * 1000 * 1000 // 8MHz
    );
    if (!ret)
    {
        printf("SPI begin failed\n");
        return;
    }

    // 2. ICM42688初期化 (CS=GPIO_NUM_40, SPI=8MHz)
    icm.begin(&spi, (gpio_num_t)40, 8 * 1000 * 1000);

    // 3. センサ用キューを用意
    g_sensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
    if (!g_sensorQueue)
    {
        printf("Failed to create sensor queue\n");
        return;
    }

    // 4. GPTimer初期化 (1MHz, alarm=1000 => 1kHz割り込み)
    static GPTimer gpt;
    if (!gpt.init(TIMER_RESOLUTION_HZ, TIMER_ALARM_COUNT))
    {
        printf("Failed to init GPTimer\n");
        return;
    }

    // 5. コールバック登録
    if (!gpt.registerCallback(sensorTimerCallback))
    {
        printf("Failed to register GPTimer callback\n");
        return;
    }

    // 6. SDMMCの初期化 (デフォルト=20MHz)
    //    HighSpeed(40MHz)にしたい場合は第1引数をtrueにしておく
    bool useHighSpeed = false; // 必要に応じて切り替え
    if (!sdCard.begin(useHighSpeed, "/sdcard", "/sdcard/gyro_log.csv"))
    {
        printf("Failed to mount or open log file.\n");
        return;
    }

    // 7. タイマー開始
    if (!gpt.start())
    {
        printf("Failed to start GPTimer\n");
        return;
    }

    // 8. オイラー角算出用クラスを初期化
    //    (ジャイロスケール=GYRO_SCALE_2000DPS, サンプリング周期=0.001s)
    static SimpleQuat quat(GYRO_SCALE_2000DPS, 0.001f);

    // キャリブレーション(バイアス取り) - 例: 1kHzで60秒
    printf("Start gyro bias calibration for 60sec...\n");
    const int sample_count = 1000 * 60; // 60秒
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int countSample = 0;

    for (int i = 0; i < sample_count; i++)
    {
        sensor_data_t data;
        if (xQueueReceive(g_sensorQueue, &data, portMAX_DELAY) == pdTRUE)
        {
            // 角速度 raw (sensor[3..5])
            sumX += data.sensor[3];
            sumY += data.sensor[4];
            sumZ += data.sensor[5];
            countSample++;
        }
    }

    // バイアス設定
    float offsetX = (countSample > 0) ? (sumX / countSample) : 0.0f;
    float offsetY = (countSample > 0) ? (sumY / countSample) : 0.0f;
    float offsetZ = (countSample > 0) ? (sumZ / countSample) : 0.0f;
    quat.setGyroBias(offsetX, offsetY, offsetZ);

    printf("Calibration done! Gyro bias = (%.2f, %.2f, %.2f)\n",
           offsetX, offsetY, offsetZ);

    // 9. メインループ: センサデータを受信してオイラー角計算し、CSVに書き込み
    while (true)
    {
        sensor_data_t recvData;
        if (xQueueReceive(g_sensorQueue, &recvData, portMAX_DELAY) == pdTRUE)
        {
            // 角速度 (sensor[3..5]) をクォータニオンに反映
            quat.updateFromRawGyro(&recvData.sensor[3]);

            // オイラー角を取得(ラジアン)
            float euler[3];
            quat.getEulerRad(euler);

            // 度数に変換
            float roll_deg = euler[0] * (180.0f / 3.1415926535f);
            float pitch_deg = euler[1] * (180.0f / 3.1415926535f);
            float yaw_deg = euler[2] * (180.0f / 3.1415926535f);

            // ログファイルに書き込み: (timestamp, x-gyro, y-gyro, z-gyro, roll, pitch, yaw)
            sdCard.writeLog(
                recvData.timestamp_us,
                recvData.sensor[3],
                recvData.sensor[4],
                recvData.sensor[5],
                roll_deg,
                pitch_deg,
                yaw_deg);

            // 動作確認用に、100回に1回だけコンソールに出力
            if (printcount++ == 100)
            {
                printf("[%.2f ms] GYRO RAW=(%d,%d,%d), EULER=(%.2f, %.2f, %.2f)\n",
                       recvData.timestamp_us / 1000.0,
                       recvData.sensor[3],
                       recvData.sensor[4],
                       recvData.sensor[5],
                       roll_deg, pitch_deg, yaw_deg);
                printcount = 0;
            }
        }
    }

    // 実際にはアプリ終了時に呼び出すこと
    // sdCard.end();
}
