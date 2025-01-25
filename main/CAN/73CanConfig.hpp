
#pragma once

#include <cstdint>

/**
 * @brief 受信データを受け取る際の構造体
 * @param id: 11bit標準ID
 * @param dlc: データ長
 * @param data: 受信データ
 */
struct CanRxFrame
{
    uint16_t id; ///< 11bit標準ID
    uint8_t dlc;
    uint8_t data[8];
};

// 送信基板ID(上位3bit)の定義
enum class BoardID : uint8_t
{
    COM = 0b000,     // 通信基板
    PARA = 0b001,    // 開放基板
    POWER = 0b010,   // 電源管理基板
    GIMBAL = 0b011,  // ジンバル基板
    CAMERA = 0b100,  // カメラ基板
    UNKNOWN = 0b111, // 不明
};

// 通信内容ID(下位8bit)の定義
enum class ContentID : uint8_t
{
    MODE_TRANSITION = 0x00, // モード遷移コマンド
    KAIHOU_COMMAND = 0x01,  // 開放機構動作コマンド
    BOARD_STATE = 0x02,     // 基板状態(要求 or 送信)
    LIFTOFF_APOGEE = 0x03,  // 離床 or 頂点検知通知
    VOLTAGE = 0x04,         // 電圧送信
    QUATERNION = 0x05,      // クオータニオン送信
};

// モード遷移コマンド(通信内容ID:0x00)
enum class ModeCommand : uint8_t
{
    START = 's',       // "Start Mode"
    PREPARATION = 'p', // "Preparation Mode"
    LOGGING = 'l',     // "Logging Mode"
};

// 開放機構動作コマンド(通信内容ID:0x01)
enum class ServoCommand : uint8_t
{
    CLOSE_SERVO = 'c',       // サーボクローズ
    ANGLE_MINUS_1 = 'n',     // パラシュートオープンの角度を1度小さくする(Startモード時のみ)
    ANGLE_PLUS_1 = 'm',      // パラシュートオープンの角度を1度大きくする(Startモード時のみ)
    CLOSE_ANGLE_PLUS = 'q',  // パラシュートクローズの角度を1度大きくする(Startモード時のみ)
    CLOSE_ANGLE_MINUS = 'r', // パラシュートクローズの角度を1度小さくする(Startモード時のみ)
};

// 離床 or 頂点検知通知(通信内容ID:0x03)
enum class ParauStatus : uint8_t
{
    APOGEE_DETECTED = 'y', // 頂点検知通知
    LIFTOFF_DETECT = 'x',  // 離床検知通知
};
