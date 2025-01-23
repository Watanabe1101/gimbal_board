#pragma once
#ifndef SDMMC_LOGGER_H
#define SDMMC_LOGGER_H

#include <unistd.h> // ★追加: fsync用
#include <cstdio>
#include <cstring>
#include <string>
#include "esp_log.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_heap_caps.h"

class SdmmcLogger
{
private:
    sdmmc_card_t *card = nullptr;
    FILE *fp = nullptr;
    bool mounted = false;
    bool highSpeed = false;
    std::string mount_point = "/sdcard";
    std::string log_path = "/sdcard/log.csv";
    uint32_t freq_khz = SDMMC_FREQ_DEFAULT;

    // ▼ DMA対応のバッファを確保して使う例
    //   4KB程度(4096バイト)以上にすると書き込みの効率が上がる
    //   MALLOC_CAP_DMA: DMA対応領域へ確保
    static constexpr size_t LOG_BUFFER_SIZE = 4 * 1024;
    char *dmaBuffer = nullptr;

public:
    bool begin(bool useHighSpeed = false,
               const char *mountPoint = "/sdcard",
               const char *logFile = "/sdcard/log.csv",
               // 追加: ピンをユーザーが指定したい場合
               int gpio_clk = GPIO_NUM_43,
               int gpio_cmd = GPIO_NUM_44,
               int gpio_d0 = GPIO_NUM_2,
               int gpio_d1 = GPIO_NUM_1,
               int gpio_d2 = GPIO_NUM_41,
               int gpio_d3 = GPIO_NUM_42)
    {
        if (mounted)
        {
            ESP_LOGW("SDMMC", "Already mounted");
            return true;
        }

        mount_point = mountPoint;
        log_path = logFile;
        highSpeed = useHighSpeed;
        freq_khz = highSpeed ? SDMMC_FREQ_HIGHSPEED : SDMMC_FREQ_DEFAULT;

        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = freq_khz;

        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        // ▼ユーザーが指定したピンを設定
        slot_config.clk = (gpio_num_t)gpio_clk;
        slot_config.cmd = (gpio_num_t)gpio_cmd;
        slot_config.d0 = (gpio_num_t)gpio_d0;
        slot_config.d1 = (gpio_num_t)gpio_d1;
        slot_config.d2 = (gpio_num_t)gpio_d2;
        slot_config.d3 = (gpio_num_t)gpio_d3;
        slot_config.width = 4;
        slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
            .disk_status_check_enable = false};

        ESP_LOGI("SDMMC", "Mounting SD card at %s (freq=%.2f MHz)...",
                 mount_point.c_str(), freq_khz / 1000.0f);

        esp_err_t ret = esp_vfs_fat_sdmmc_mount(
            mount_point.c_str(),
            &host,
            &slot_config,
            &mount_config,
            &card);
        if (ret != ESP_OK)
        {
            ESP_LOGE("SDMMC", "Mount failed (0x%x)", ret);
            return false;
        }
        mounted = true;
        sdmmc_card_print_info(stdout, card);

        // ログファイルを開く
        fp = fopen(log_path.c_str(), "w");
        if (!fp)
        {
            ESP_LOGE("SDMMC", "Failed to open log file: %s", log_path.c_str());
            return false;
        }
        ESP_LOGI("SDMMC", "Log file opened: %s", log_path.c_str());

        // ▼ DMA対応領域へ大きめのバッファを確保し、setvbuf() に設定
        dmaBuffer = (char *)heap_caps_malloc(LOG_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (dmaBuffer)
        {
            // _IOFBF: 完全バッファリング、LOG_BUFFER_SIZE: バッファサイズ
            setvbuf(fp, dmaBuffer, _IOFBF, LOG_BUFFER_SIZE);
            ESP_LOGI("SDMMC", "Enabled DMA buffer for file (size=%d)", LOG_BUFFER_SIZE);
        }
        else
        {
            ESP_LOGW("SDMMC", "Failed to alloc DMA buffer. Using default buffer.");
        }

        // CSVヘッダ等を書いておく
        fprintf(fp, "timestamp(us),x-gyro,y-gyro,z-gyro,roll,pitch,yaw\n");

        return true;
    }

    void end()
    {
        if (fp)
        {
            fclose(fp);
            fp = nullptr;
        }
        if (mounted)
        {
            esp_vfs_fat_sdcard_unmount(mount_point.c_str(), card);
            mounted = false;
        }
        // 確保したバッファを解放
        if (dmaBuffer)
        {
            heap_caps_free(dmaBuffer);
            dmaBuffer = nullptr;
        }
        ESP_LOGI("SDMMC", "Unmounted SD card");
    }

    // ログ書き込み
    void writeLog(uint64_t timestampUs,
                  int16_t xGyroRaw, int16_t yGyroRaw, int16_t zGyroRaw,
                  float rollDeg, float pitchDeg, float yawDeg)
    {
        if (!fp)
            return;
        fprintf(fp, "%llu,%d,%d,%d,%.4f,%.4f,%.4f\n",
                (long long unsigned)timestampUs,
                xGyroRaw, yGyroRaw, zGyroRaw,
                rollDeg, pitchDeg, yawDeg);
    }

    // fflush()のラッパ (必要に応じて呼び出し)
    void flush()
    {
        if (!fp)
            return;

        // ★ 1) fflushでライブラリバッファをクリア
        fflush(fp);

        // ★ 2) fsyncでOSレベルのキャッシュを物理メディアへ書き込み
        int fd = fileno(fp); // ファイルディスクリプタ取得
        if (fd >= 0)
        {
            fsync(fd);
        }
    }
};

#endif // SDMMC_LOGGER_H
