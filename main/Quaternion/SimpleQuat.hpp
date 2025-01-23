#pragma once

#include <stdint.h>
#include <math.h>

/**
 * @brief クォータニオン姿勢推定クラス
 *
 * - 内部は float で計算
 * - ICM42688などの生データ (int16_t) を直接受け取り、内部でスケーリングして rad/s に変換
 * - 1ms周期(例) など固定のサンプリング周期を前提とし、ジャイロのみでクォータニオンを更新
 */
class SimpleQuat
{
public:
    /**
     * @brief コンストラクタ
     *
     * @param gyro_scale_factor  生データ(int16_t) -> [rad/s] へ変換する際のスケーリング係数
     * @param dt_sec             サンプリング周期 [s] (例: 0.001f で 1ms)
     */
    SimpleQuat(float gyro_scale_factor, float dt_sec)
        : gyro_scale_factor_(gyro_scale_factor), dt_sec_(dt_sec)
    {
        reset();
    }

    /**
     * @brief クォータニオンを初期化(単位元にリセット)する
     */
    void reset()
    {
        // w=1, x=y=z=0
        q_[0] = 1.0f;
        q_[1] = 0.0f;
        q_[2] = 0.0f;
        q_[3] = 0.0f;
    }

    /**
     * @brief 外部で推定したジャイロバイアスをセット
     * @param bx,by,bz 各軸のバイアス（生データ値 int16_t相当でOK）
     *
     *  - ここでは生データ値そのもの(物理値への換算前)を想定
     */
    void setGyroBias(float bx, float by, float bz)
    {
        bias_[0] = bx;
        bias_[1] = by;
        bias_[2] = bz;
    }

    /**
     * @brief ジャイロの生データ (int16_t) を受け取り、内部で[rad/s]に換算 → クォータニオンを更新
     *
     * @param raw_gyro  int16_t型の配列 [3] (X, Y, Z) の順でジャイロ生データを想定
     */
    void updateFromRawGyro(const int16_t raw_gyro[3])
    {
        // 1. (生データ - バイアス) -> [rad/s] へスケーリング
        float gx = (raw_gyro[0] - bias_[0]) * gyro_scale_factor_; // [rad/s]
        float gy = (raw_gyro[1] - bias_[1]) * gyro_scale_factor_;
        float gz = (raw_gyro[2] - bias_[2]) * gyro_scale_factor_;

        // 2. クォータニオン更新
        //    dq/dt = 0.5 * Omega(gyro) * q
        //    離散化: q_new = q_old + 0.5 * dt * Omega(gyro)*q_old
        float dt = dt_sec_;
        float rotate[4][4] = {
            {0.0f, -gx, -gy, -gz},
            {gx, 0.0f, gz, -gy},
            {gy, -gz, 0.0f, gx},
            {gz, gy, -gx, 0.0f}};

        float qnew[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++)
        {
            // 元のq_をベースに更新
            qnew[i] = q_[i];
            for (int k = 0; k < 4; k++)
            {
                qnew[i] += 0.5f * dt * rotate[i][k] * q_[k];
            }
        }

        // 3. 正規化
        float mag = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            mag += qnew[i] * qnew[i];
        }
        mag = sqrtf(mag);
        if (mag > 1e-8f)
        {
            for (int i = 0; i < 4; i++)
            {
                q_[i] = qnew[i] / mag;
            }
        }
    }

    /**
     * @brief 現在のクォータニオンからオイラー角 [roll, pitch, yaw] を取得 (単位=ラジアン)
     *
     * @param euler_rad [out] float配列[3] へ (roll, pitch, yaw) [rad] を格納
     *
     *  - roll = x軸周り回転
     *  - pitch= y軸周り回転
     *  - yaw  = z軸周り回転
     *
     */
    void getEulerRad(float euler_rad[3]) const
    {
        // ここではZ-Y-X系(= yaw-pitch-roll)の一例
        // ただし実装や用途によって変化あり

        const float w = q_[0];
        const float x = q_[1];
        const float y = q_[2];
        const float z = q_[3];

        // pitch = asin(2*(w*x - y*z))
        float sinp = 2.0f * (w * x - y * z);
        if (sinp > 1.0f)
            sinp = 1.0f;
        if (sinp < -1.0f)
            sinp = -1.0f;
        float pitch = asinf(sinp);

        // roll = atan2(2(w*y + x*z), w^2 + z^2 - x^2 - y^2)
        float roll = atan2f(2.0f * (w * y + x * z),
                            w * w + z * z - x * x - y * y);

        // yaw = atan2(2(w*z + x*y), w^2 + x^2 - y^2 - z^2)
        float yaw = atan2f(2.0f * (w * z + x * y),
                           w * w + x * x - y * y - z * z);

        euler_rad[0] = roll;
        euler_rad[1] = pitch;
        euler_rad[2] = yaw;
    }

    /**
     * @brief 現在のクォータニオン (w, x, y, z) を取得
     * @param quat_out [out] float配列[4] へクォータニオンを格納
     */
    void GetQuarternion(float quat_out[4]) const
    {
        quat_out[0] = q_[0]; // w
        quat_out[1] = q_[1]; // x
        quat_out[2] = q_[2]; // y
        quat_out[3] = q_[3]; // z
    }

private:
    float q_[4];              ///< クォータニオン (w, x, y, z)
    float gyro_scale_factor_; ///< 生データ(int16)→[rad/s]変換係数
    float dt_sec_;            ///< サンプリング周期 [s]

    float bias_[3] = {0, 0, 0}; ///< ジャイロオフセット（生データ値そのもの）
};
