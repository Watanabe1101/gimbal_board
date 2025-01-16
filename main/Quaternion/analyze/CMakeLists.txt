cmake_minimum_required(VERSION 3.15)
project(simplequat_python_example)

# pybind11を探す (systemにインストール済み想定)
find_package(pybind11 REQUIRED)

# 他にもC++のバージョン指定など
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# simplequatpyというPythonモジュールをビルド
pybind11_add_module(simplequatpy
    pybind_simplequat.cpp       # Pythonラッパ
    # 必要に応じて SimpleQuat.cpp があれば追加
)

# ヘッダパスを追加するなら
target_include_directories(simplequatpy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
