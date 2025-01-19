#include <pybind11/pybind11.h>
#include <pybind11/stl.h>    
#include <pybind11/numpy.h>   

#include "SimpleQuat.h"

// pybind11の名前空間
namespace py = pybind11;

/**
 * Python側に公開するラッパークラス
 */
class PySimpleQuat {
public:
    // コンストラクタ
    PySimpleQuat(float gyro_scale_factor, float dt_sec)
        : quat_(gyro_scale_factor, dt_sec)
    {
    }

    // クォータニオンのリセット
    void reset() {
        quat_.reset();
    }

    // ジャイロ生データを更新
    void updateFromRawGyro(const std::vector<int16_t>& raw_gyro) {
        // raw_gyroが [X, Y, Z] の3要素を想定
        if(raw_gyro.size() != 3){
            throw std::runtime_error("raw_gyro must have 3 elements (X, Y, Z)");
        }
        quat_.updateFromRawGyro(raw_gyro.data());
    }

    // (roll, pitch, yaw) [rad] を返す
    std::vector<float> getEulerRad() const {
        float euler[3];
        quat_.getEulerRad(euler);
        return {euler[0], euler[1], euler[2]};
    }

private:
    SimpleQuat quat_;
};

/**
 * Pythonモジュールの定義
 */
PYBIND11_MODULE(simplequatpy, m) {
    m.doc() = "SimpleQuat wrapper for python (using pybind11)";

    // PySimpleQuat クラスを Python側に公開
    py::class_<PySimpleQuat>(m, "SimpleQuat")
        .def(py::init<float, float>(),
             py::arg("gyro_scale_factor"),
             py::arg("dt_sec")=0.001f,
             R"pbdoc(
                Constructor for SimpleQuat
                :param gyro_scale_factor: int16->rad/s変換スケール
                :param dt_sec: サンプリング周期[s]
             )pbdoc")
        .def("reset", &PySimpleQuat::reset)
        .def("updateFromRawGyro", &PySimpleQuat::updateFromRawGyro,
             R"pbdoc(
                Update quaternion with raw gyro data (int16 x3)
                :param raw_gyro: [gx_int16, gy_int16, gz_int16]
             )pbdoc")
        .def("getEulerRad", &PySimpleQuat::getEulerRad,
             R"pbdoc(
                Get euler angles [roll, pitch, yaw] in radians
                :return: list of 3 floats
             )pbdoc");
}
