#pragma once
#ifndef SDMMC_LOGGER_H
#define SDMMC_LOGGER_H

#include <cstdio>
#include <cstring>
#include <string>
#include "esp_log.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

/**
 * @class SdmmcLogger
 * SDMMCを使ったロギング用クラス
 */
class SdmmcLogger
{
private:
    sdmmc_card_t *card = nullptr;             // SDカード情報
    FILE *fp = nullptr;                       // ログファイルのFILEポインタ
    bool mounted = false;                     // マウント済みかどうか
    bool highSpeed = false;                   // ハイスピードモードかどうか
    std::string mount_point = "/sdcard";      // マウント先パス
    std::string log_path = "/sdcard/log.csv"; // ログファイルパス

    // 現在の設定周波数(kHz)
    // デフォルトは 20MHz (SDMMC_FREQ_DEFAULT = 20000)
    uint32_t freq_khz = SDMMC_FREQ_DEFAULT;

public:
    /**
     * @brief begin() - SDMMC初期化＆マウント
     * @param useHighSpeed trueなら40MHzを使う(ハイスピードモード)
     * @param mountPoint   マウント先のパス(例: "/sdcard")
     * @param logFile      ログファイルのパス(例: "/sdcard/log.csv")
     * @return true: 成功, false: 失敗
     */
    bool begin(bool useHighSpeed = false,
               const char *mountPoint = "/sdcard",
               const char *logFile = "/sdcard/log.csv")
    {
        // 1) 既にマウント済みなら何もしない
        if (mounted)
        {
            ESP_LOGW("SDMMC", "Already mounted");
            return true;
        }

        // 2) パラメータ設定
        mount_point = mountPoint;
        log_path = logFile;
        highSpeed = useHighSpeed;
        freq_khz = highSpeed ? SDMMC_FREQ_HIGHSPEED : SDMMC_FREQ_DEFAULT;

        // 3) ホスト＆スロットの設定
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = freq_khz; // 20MHz or 40MHz

        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        // ピン割り当て
        slot_config.clk = GPIO_NUM_43;
        slot_config.cmd = GPIO_NUM_44;
        slot_config.d0 = GPIO_NUM_2;
        slot_config.d1 = GPIO_NUM_1;
        slot_config.d2 = GPIO_NUM_41;
        slot_config.d3 = GPIO_NUM_42;

        slot_config.width = 4; // 4ビットバス
        // 内部プルアップ有効 (外付け推奨)
        slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        // 4) マウント設定
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
            .disk_status_check_enable = false};

        ESP_LOGI("SDMMC", "Mounting SD card at %s (freq=%.2f MHz)...",
                 mount_point.c_str(), freq_khz / 1000.0f);

        // 5) マウント実行
        esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point.c_str(),
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

        // カード情報表示
        sdmmc_card_print_info(stdout, card);

        // 6) ログファイルを開く (w:上書き)
        fp = fopen(log_path.c_str(), "w");
        if (!fp)
        {
            ESP_LOGE("SDMMC", "Failed to open log file: %s", log_path.c_str());
            return false;
        }
        ESP_LOGI("SDMMC", "Log file opened: %s", log_path.c_str());

        // 7) ヘッダ行などを書いておきたい場合はここで
        fprintf(fp, "timestamp(us),x-gyro,y-gyro,z-gyro,roll,pitch,yaw\n");

        return true;
    }

    /**
     * @brief end() - ログファイルをクローズ＆アンマウント
     */
    void end()
    {
        if (fp)
        {
            fclose(fp);
            fp = nullptr;
        }
        if (mounted)
        {
            esp_vfs_fat_sdmmc_unmount();
            mounted = false;
        }
        ESP_LOGI("SDMMC", "Unmounted SD card");
    }

    /**
     * @brief writeLog() - ジャイロ、姿勢角をCSV形式でログ書き込み
     * @param timestampUs   取得時刻(μs)
     * @param xGyroRaw      x軸ジャイロ生データ
     * @param yGyroRaw      y軸ジャイロ生データ
     * @param zGyroRaw      z軸ジャイロ生データ
     * @param rollDeg       ロール角度(deg)
     * @param pitchDeg      ピッチ角度(deg)
     * @param yawDeg        ヨー角度(deg)
     */
    void writeLog(uint64_t timestampUs,
                  int16_t xGyroRaw, int16_t yGyroRaw, int16_t zGyroRaw,
                  float rollDeg, float pitchDeg, float yawDeg)
    {
        if (!fp)
        {
            // 未オープンやエラー時は無視
            return;
        }
        fprintf(fp, "%llu,%d,%d,%d,%.4f,%.4f,%.4f\n",
                (long long unsigned)timestampUs,
                xGyroRaw, yGyroRaw, zGyroRaw,
                rollDeg, pitchDeg, yawDeg);
    }
};

#endif // SDMMC_LOGGER_H
