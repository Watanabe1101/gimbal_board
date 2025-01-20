#include <stdio.h>
#include <cstdio>
#include <string>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"

// sdk関連
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "foc/esp_foc.h"
#include "svpwm/esp_svpwm.h"
#include "SPICREATE.h"
#include "ICM42688.h"
#include "gptimer.h"
#include "SimpleQuat.h"

// ログタグ
static const char *TAG = "SDMMC_Example";

extern "C" void app_main(void)
{
    // 1) SDMMCホストの設定
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // High Speedモード(40 MHz)を使うため設定
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz

    // 2) スロット設定
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // ピン割り当て
    slot_config.clk = GPIO_NUM_43; // CLK
    slot_config.cmd = GPIO_NUM_44; // CMD
    slot_config.d0 = GPIO_NUM_2;   // D0
    slot_config.d1 = GPIO_NUM_1;   // D1
    slot_config.d2 = GPIO_NUM_41;  // D2
    slot_config.d3 = GPIO_NUM_42;  // D3

    // 4ラインで使用（カードが4ビット幅をサポートしている想定）
    slot_config.width = 4;

    // 内部プルアップを有効化（※実運用では外付け推奨）
    // フラグに SDMMC_SLOT_FLAG_INTERNAL_PULLUP を追加
    slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // 3) SDカードをVFSにマウント
    //    mount_config でマウント時の動作を制御
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,   // マウントに失敗してもフォーマットしない
        .max_files = 5,                    // 同時オープンできるファイル数
        .allocation_unit_size = 16 * 1024, // FATのアロケーション単位(パフォーマンス調整用)
        .disk_status_check_enable = false};

    // SDカード情報を受け取るための構造体
    sdmmc_card_t *card;

    ESP_LOGI(TAG, "Mounting SD card...");

    // 実際にマウントを実行
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card (error: 0x%x)", ret);
        return;
    }

    // 正常にマウントできた場合、SDカードの情報をログに出す
    sdmmc_card_print_info(stdout, card);

    // 4) ファイルI/Oのテスト（書き込みテスト）
    const char *filePath = "/sdcard/test.txt";
    FILE *f = fopen(filePath, "w");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
    }
    else
    {
        fprintf(f, "Hello SD card from ESP32-S3 at 40 MHz!\n");
        fclose(f);
        ESP_LOGI(TAG, "File written: %s", filePath);
    }

    // 5) ファイルを再度読み込みテスト
    f = fopen(filePath, "r");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
    }
    else
    {
        char line[128];
        if (fgets(line, sizeof(line), f) != nullptr)
        {
            ESP_LOGI(TAG, "File read: %s", line);
        }
        fclose(f);
    }

    // 6) もしSDカードをアンマウントしたい場合は:
    // esp_vfs_fat_sdmmc_unmount();
    // SDMMCホストを停止したい場合は sdmmc_host_deinit() を呼ぶ

    // 実行が継続するので、適当にタスクをdelay
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}