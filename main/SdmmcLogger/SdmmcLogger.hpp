#pragma once
#ifndef SDMMC_LOGGER_H
#define SDMMC_LOGGER_H

#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include "esp_log.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief Enhanced SdmmcLogger with modular data logging capabilities
 *
 * This class provides flexible logging to SD card with support for:
 * - IMU data (gyro, acceleration)
 * - Motor data (encoder positions, targets)
 * - System status information
 * - Custom data fields
 *
 * Features:
 * - Thread-safe logging with mutexes
 * - Efficient DMA buffering
 * - Dynamic column configuration
 * - Multiple file support
 * - Auto file management (automatic file numbering for existing files)
 * - Separate files for different data types
 */
class SdmmcLogger
{
public:
    // Log data types for modular logging
    enum class DataType
    {
        TIMESTAMP,      // Timestamp (microseconds)
        GYRO,           // Gyroscope data (x,y,z)
        ACCEL,          // Accelerometer data (x,y,z)
        EULER,          // Euler angles (roll,pitch,yaw)
        QUATERNION,     // Quaternion (w,x,y,z)
        MOTOR_ENCODER,  // Motor encoder position
        MOTOR_VELOCITY, // Motor velocity
        MOTOR_CURRENT,  // Motor current
        MOTOR_TARGET,   // Motor target position/velocity
        CUSTOM          // Custom data fields
    };

    // Configuration structure
    struct Config
    {
        bool useHighSpeed = false;
        std::string mountPoint = "/sdcard";
        std::string logFile = "/sdcard/log.csv";
        std::string motorLogFile = "/sdcard/motor_log.csv"; // モーターログファイル
        int gpio_clk = GPIO_NUM_43;
        int gpio_cmd = GPIO_NUM_44;
        int gpio_d0 = GPIO_NUM_2;
        int gpio_d1 = GPIO_NUM_1;
        int gpio_d2 = GPIO_NUM_41;
        int gpio_d3 = GPIO_NUM_42;
        size_t bufferSize = 4 * 1024;   // 4KB default buffer
        bool autoFlush = false;         // Auto flush after each write
        uint32_t autoFlushInterval = 0; // Auto flush interval in milliseconds (0 = disabled)
        bool autoNumberFiles = true;    // 同名ファイルがある場合に自動的に番号を付与
    };

    // Column definition for log files
    struct ColumnDef
    {
        std::string name;
        DataType type;
        int index;          // For arrays (e.g., index=0 for gyro_x)
        std::string format; // Printf-style format string
    };

private:
    sdmmc_card_t *card = nullptr;
    FILE *fp = nullptr;
    FILE *motorFp = nullptr; // 別ファイルでモーターデータを記録
    bool mounted = false;
    bool highSpeed = false;
    std::string mount_point = "/sdcard";
    std::string log_path = "/sdcard/log.csv";
    std::string motor_log_path = "/sdcard/motor_log.csv"; // モーターログ用のパス
    uint32_t freq_khz = SDMMC_FREQ_DEFAULT;

    // Buffer for efficient writing
    size_t bufferSize = 4 * 1024;
    char *dmaBuffer = nullptr;
    char *motorDmaBuffer = nullptr; // モーターログ用のバッファ

    // Thread safety
    SemaphoreHandle_t fileMutex = nullptr;
    SemaphoreHandle_t motorFileMutex = nullptr; // モーターファイル用のミューテックス

    // Column definitions
    std::vector<ColumnDef> columns;
    std::vector<ColumnDef> motorColumns; // モーター用のカラム定義

    // Auto flush settings
    bool autoFlush = false;
    uint32_t autoFlushCounter = 0;
    uint32_t autoFlushInterval = 100;
    uint32_t motorFlushCounter = 0; // モーターログ用のフラッシュカウンター

public:
    SdmmcLogger()
    {
        // Create mutexes for thread safety
        fileMutex = xSemaphoreCreateMutex();
        motorFileMutex = xSemaphoreCreateMutex();
    }

    ~SdmmcLogger()
    {
        end();
        if (fileMutex)
        {
            vSemaphoreDelete(fileMutex);
            fileMutex = nullptr;
        }
        if (motorFileMutex)
        {
            vSemaphoreDelete(motorFileMutex);
            motorFileMutex = nullptr;
        }
    }

    /**
     * @brief ファイル名に番号を付与する（既存ファイルがある場合）
     * @param basePath 基本ファイルパス
     * @return 使用可能なファイルパス
     */
    std::string generateUniqueFilename(const std::string &basePath)
    {
        if (access(basePath.c_str(), F_OK) != 0)
        {
            // ファイルが存在しない場合はそのまま返す
            return basePath;
        }

        // ファイル名と拡張子を分離
        std::string baseDir;
        std::string baseFileName;
        std::string extension;

        size_t lastSlash = basePath.find_last_of('/');
        if (lastSlash != std::string::npos)
        {
            baseDir = basePath.substr(0, lastSlash + 1);
            baseFileName = basePath.substr(lastSlash + 1);
        }
        else
        {
            baseDir = "";
            baseFileName = basePath;
        }

        size_t lastDot = baseFileName.find_last_of('.');
        if (lastDot != std::string::npos)
        {
            extension = baseFileName.substr(lastDot);
            baseFileName = baseFileName.substr(0, lastDot);
        }
        else
        {
            extension = "";
        }

        // 連番を試して、存在しないファイル名を見つける
        for (int counter = 1; counter < 100; counter++)
        { // 最大99まで
            std::string newPath = baseDir + baseFileName + "_" + std::to_string(counter) + extension;
            if (access(newPath.c_str(), F_OK) != 0)
            {
                // ファイルが存在しない場合はこのパスを使用
                return newPath;
            }
        }

        // タイムスタンプを付与
        char timestamp[20];
        time_t now = time(NULL);
        strftime(timestamp, sizeof(timestamp), "_%Y%m%d_%H%M%S", localtime(&now));
        return baseDir + baseFileName + timestamp + extension;
    }

    /**
     * @brief Initialize the SD card and log file with default configuration
     * @param config Logger configuration
     * @return true if successful, false otherwise
     */
    bool begin(const Config &config)
    {
        if (mounted)
        {
            ESP_LOGW("SDMMC", "Already mounted");
            return true;
        }

        // Copy configuration
        mount_point = config.mountPoint;
        log_path = config.logFile;
        motor_log_path = config.motorLogFile;
        highSpeed = config.useHighSpeed;
        freq_khz = highSpeed ? SDMMC_FREQ_HIGHSPEED : SDMMC_FREQ_DEFAULT;
        bufferSize = config.bufferSize;
        autoFlush = config.autoFlush;
        autoFlushInterval = config.autoFlushInterval;

        // Configure SDMMC host
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = freq_khz;

        // Configure slot
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        slot_config.clk = (gpio_num_t)config.gpio_clk;
        slot_config.cmd = (gpio_num_t)config.gpio_cmd;
        slot_config.d0 = (gpio_num_t)config.gpio_d0;
        slot_config.d1 = (gpio_num_t)config.gpio_d1;
        slot_config.d2 = (gpio_num_t)config.gpio_d2;
        slot_config.d3 = (gpio_num_t)config.gpio_d3;
        slot_config.width = 4;
        slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        // Mount configuration
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
            .disk_status_check_enable = false};

        ESP_LOGI("SDMMC", "Mounting SD card at %s (freq=%.2f MHz)...",
                 mount_point.c_str(), freq_khz / 1000.0f);

        // Mount the SD card
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

        // 既存ファイルがある場合は新しいファイル名を生成
        if (config.autoNumberFiles)
        {
            log_path = generateUniqueFilename(log_path);
            motor_log_path = generateUniqueFilename(motor_log_path);
        }

        // IMUログファイルを開く
        fp = fopen(log_path.c_str(), "w");
        if (!fp)
        {
            ESP_LOGE("SDMMC", "Failed to open IMU log file: %s", log_path.c_str());
            return false;
        }
        ESP_LOGI("SDMMC", "IMU log file opened: %s", log_path.c_str());

        // モーターログファイルを開く
        motorFp = fopen(motor_log_path.c_str(), "w");
        if (!motorFp)
        {
            ESP_LOGE("SDMMC", "Failed to open motor log file: %s", motor_log_path.c_str());
            fclose(fp);
            fp = nullptr;
            return false;
        }
        ESP_LOGI("SDMMC", "Motor log file opened: %s", motor_log_path.c_str());

        // IMUログ用DMAバッファを確保
        dmaBuffer = (char *)heap_caps_malloc(bufferSize, MALLOC_CAP_DMA);
        if (dmaBuffer)
        {
            setvbuf(fp, dmaBuffer, _IOFBF, bufferSize);
            ESP_LOGI("SDMMC", "Enabled DMA buffer for IMU file (size=%zu)", bufferSize);
        }
        else
        {
            ESP_LOGW("SDMMC", "Failed to alloc DMA buffer for IMU log. Using default buffer.");
        }

        // モーターログ用DMAバッファを確保
        motorDmaBuffer = (char *)heap_caps_malloc(bufferSize, MALLOC_CAP_DMA);
        if (motorDmaBuffer)
        {
            setvbuf(motorFp, motorDmaBuffer, _IOFBF, bufferSize);
            ESP_LOGI("SDMMC", "Enabled DMA buffer for motor file (size=%zu)", bufferSize);
        }
        else
        {
            ESP_LOGW("SDMMC", "Failed to alloc DMA buffer for motor log. Using default buffer.");
        }

        // モーターログのヘッダーを書き込み
        fprintf(motorFp, "timestamp_us,encoder1_pos,target1_pos,encoder2_pos,target2_pos\n");

        return true;
    }

    /**
     * @brief Initialize with default settings
     */
    bool begin(bool useHighSpeed = false,
               const char *mountPoint = "/sdcard",
               const char *logFile = "/sdcard/log.csv",
               const char *motorLogFile = "/sdcard/motor_log.csv",
               int gpio_clk = GPIO_NUM_43,
               int gpio_cmd = GPIO_NUM_44,
               int gpio_d0 = GPIO_NUM_2,
               int gpio_d1 = GPIO_NUM_1,
               int gpio_d2 = GPIO_NUM_41,
               int gpio_d3 = GPIO_NUM_42)
    {

        Config config;
        config.useHighSpeed = useHighSpeed;
        config.mountPoint = mountPoint;
        config.logFile = logFile;
        config.motorLogFile = motorLogFile;
        config.gpio_clk = gpio_clk;
        config.gpio_cmd = gpio_cmd;
        config.gpio_d0 = gpio_d0;
        config.gpio_d1 = gpio_d1;
        config.gpio_d2 = gpio_d2;
        config.gpio_d3 = gpio_d3;
        config.autoNumberFiles = true; // デフォルトで有効化

        return begin(config);
    }

    /**
     * @brief Close the log file and unmount the SD card
     */
    void end()
    {
        // IMUログファイルの終了処理
        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            if (fp)
            {
                fflush(fp);
                fclose(fp);
                fp = nullptr;
            }

            if (dmaBuffer)
            {
                heap_caps_free(dmaBuffer);
                dmaBuffer = nullptr;
            }

            xSemaphoreGive(fileMutex);
        }

        // モーターログファイルの終了処理
        if (xSemaphoreTake(motorFileMutex, portMAX_DELAY) == pdTRUE)
        {
            if (motorFp)
            {
                fflush(motorFp);
                fclose(motorFp);
                motorFp = nullptr;
            }

            if (motorDmaBuffer)
            {
                heap_caps_free(motorDmaBuffer);
                motorDmaBuffer = nullptr;
            }

            xSemaphoreGive(motorFileMutex);
        }

        // SDカードのアンマウント
        if (mounted)
        {
            esp_vfs_fat_sdcard_unmount(mount_point.c_str(), card);
            mounted = false;
            ESP_LOGI("SDMMC", "Unmounted SD card");
        }
    }

    /**
     * @brief Set up columns for logging with automatic header generation
     * @param newColumns Vector of column definitions
     * @return true if successful, false otherwise
     */
    bool setColumns(const std::vector<ColumnDef> &newColumns)
    {
        if (!fp || newColumns.empty())
        {
            return false;
        }

        if (xSemaphoreTake(fileMutex, portMAX_DELAY) != pdTRUE)
        {
            return false;
        }

        columns = newColumns;

        // Write CSV header
        for (size_t i = 0; i < columns.size(); i++)
        {
            fprintf(fp, "%s%s", columns[i].name.c_str(), (i < columns.size() - 1) ? "," : "\n");
        }

        fflush(fp);
        xSemaphoreGive(fileMutex);
        return true;
    }

    /**
     * @brief Log IMU data (original compatibility function)
     */
    void writeLog(uint64_t timestampUs,
                  int16_t xGyroRaw, int16_t yGyroRaw, int16_t zGyroRaw,
                  float rollDeg, float pitchDeg, float yawDeg)
    {

        if (!fp)
            return;

        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            fprintf(fp, "%llu,%d,%d,%d,%.4f,%.4f,%.4f\n",
                    (long long unsigned)timestampUs,
                    xGyroRaw, yGyroRaw, zGyroRaw,
                    rollDeg, pitchDeg, yawDeg);

            if (autoFlush && (++autoFlushCounter >= autoFlushInterval))
            {
                flushInternal();
                autoFlushCounter = 0;
            }

            xSemaphoreGive(fileMutex);
        }
    }

    /**
     * @brief Log IMU data with motor encoder values
     */
    void writeLogWithEncoder(uint64_t timestampUs,
                             int16_t xGyroRaw, int16_t yGyroRaw, int16_t zGyroRaw,
                             float rollDeg, float pitchDeg, float yawDeg,
                             float encoder1Pos, float encoder2Pos)
    {

        if (!fp)
            return;

        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            fprintf(fp, "%llu,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    (long long unsigned)timestampUs,
                    xGyroRaw, yGyroRaw, zGyroRaw,
                    rollDeg, pitchDeg, yawDeg,
                    encoder1Pos, encoder2Pos);

            if (autoFlush && (++autoFlushCounter >= autoFlushInterval))
            {
                flushInternal();
                autoFlushCounter = 0;
            }

            xSemaphoreGive(fileMutex);
        }
    }

    /**
     * @brief Log comprehensive motor data
     */
    void writeMotorLog(uint64_t timestampUs,
                       float encoder1Pos, float encoder2Pos,
                       float motor1Vel, float motor2Vel,
                       float motor1Target, float motor2Target,
                       float motor1Current, float motor2Current)
    {

        if (!fp)
            return;

        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            fprintf(fp, "%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    (long long unsigned)timestampUs,
                    encoder1Pos, encoder2Pos,
                    motor1Vel, motor2Vel,
                    motor1Target, motor2Target,
                    motor1Current, motor2Current);

            if (autoFlush && (++autoFlushCounter >= autoFlushInterval))
            {
                flushInternal();
                autoFlushCounter = 0;
            }

            xSemaphoreGive(fileMutex);
        }
    }

    /**
     * @brief Generic logging function with variable arguments
     * Uses a formatting string similar to printf
     */
    void writeGeneric(const char *format, ...)
    {
        if (!fp)
            return;

        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            va_list args;
            va_start(args, format);
            vfprintf(fp, format, args);
            va_end(args);

            if (autoFlush && (++autoFlushCounter >= autoFlushInterval))
            {
                flushInternal();
                autoFlushCounter = 0;
            }

            xSemaphoreGive(fileMutex);
        }
    }

    /**
     * @brief モーターデータのみをログに記録（タイムスタンプ、エンコーダー値、目標値）
     */
    void writeMotorData(uint64_t timestampUs,
                        float encoder1Pos, float target1Pos,
                        float encoder2Pos, float target2Pos)
    {
        if (!motorFp)
            return;

        if (xSemaphoreTake(motorFileMutex, portMAX_DELAY) == pdTRUE)
        {
            fprintf(motorFp, "%llu,%.4f,%.4f,%.4f,%.4f\n",
                    (long long unsigned)timestampUs,
                    encoder1Pos, target1Pos,
                    encoder2Pos, target2Pos);

            if (autoFlush && (++motorFlushCounter >= autoFlushInterval))
            {
                flushMotorInternal();
                motorFlushCounter = 0;
            }

            xSemaphoreGive(motorFileMutex);
        }
    }

    /**
     * @brief 両方のログバッファをディスクに書き込む
     */
    void flush()
    {
        // IMUログのフラッシュ
        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            flushInternal();
            xSemaphoreGive(fileMutex);
        }

        // モーターログのフラッシュ
        if (xSemaphoreTake(motorFileMutex, portMAX_DELAY) == pdTRUE)
        {
            flushMotorInternal();
            xSemaphoreGive(motorFileMutex);
        }
    }

    /**
     * @brief IMUログのみをフラッシュ
     */
    void flushImuLog()
    {
        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
            flushInternal();
            xSemaphoreGive(fileMutex);
        }
    }

    /**
     * @brief モーターログのみをフラッシュ
     */
    void flushMotorLog()
    {
        if (xSemaphoreTake(motorFileMutex, portMAX_DELAY) == pdTRUE)
        {
            flushMotorInternal();
            xSemaphoreGive(motorFileMutex);
        }
    }

private:
    /**
     * @brief Internal flush function for IMU log (assumes mutex is already acquired)
     */
    void flushInternal()
    {
        if (!fp)
            return;

        // 1) ライブラリバッファをフラッシュ
        fflush(fp);

        // 2) OSキャッシュを物理メディアに同期
        int fd = fileno(fp);
        if (fd >= 0)
        {
            fsync(fd);
        }
    }

    /**
     * @brief Internal flush function for motor log (assumes mutex is already acquired)
     */
    void flushMotorInternal()
    {
        if (!motorFp)
            return;

        // 1) ライブラリバッファをフラッシュ
        fflush(motorFp);

        // 2) OSキャッシュを物理メディアに同期
        int fd = fileno(motorFp);
        if (fd >= 0)
        {
            fsync(fd);
        }
    }
};

#endif // SDMMC_LOGGER_H