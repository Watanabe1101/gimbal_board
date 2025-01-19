import csv
import math
import matplotlib.pyplot as plt

import simplequatpy


def main():
    # ----- ユーザー設定 -----
    csv_file = "sample_gyro.csv"

    # ±2000[deg/s] レンジ, LSB=16.4 LSB/deg/s => deg/s = raw/16.4 => rad/s = deg/s * (pi/180)
    #  => scaleFactor = (1/16.4) * (pi/180) ~ 0.001065
    gyro_scale_factor = (1.0 / 16.4) * (math.pi / 180.0)

    dt_sec = 0.001  # 1ms想定

    # ----- Quat初期化 -----
    quat = simplequatpy.SimpleQuat(gyro_scale_factor, dt_sec)

    # グラフ用リスト
    roll_list = []
    pitch_list = []
    yaw_list = []

    # CSV読み込み
    with open(csv_file, "r", newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            # コメント行や空行の排除
            if not row or row[0].startswith('#'):
                continue

            # 1行に X, Y, Z (int) が入っている想定
            gx = int(row[0])
            gy = int(row[1])
            gz = int(row[2])

            # Update quaternion
            quat.updateFromRawGyro([gx, gy, gz])

            # Get Euler angles [roll, pitch, yaw] in rad
            roll, pitch, yaw = quat.getEulerRad()

            # degに変換してログ
            roll_deg  = roll  * 180.0 / math.pi
            pitch_deg = pitch * 180.0 / math.pi
            yaw_deg   = yaw   * 180.0 / math.pi

            roll_list.append(roll_deg)
            pitch_list.append(pitch_deg)
            yaw_list.append(yaw_deg)

    # グラフ化
    t = [i*dt_sec for i in range(len(roll_list))]  # 時間[s]
    plt.figure(figsize=(10,6))
    plt.plot(t, roll_list, label="Roll (deg)")
    plt.plot(t, pitch_list, label="Pitch (deg)")
    plt.plot(t, yaw_list, label="Yaw (deg)")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.title("Quaternion Integration Result")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()
