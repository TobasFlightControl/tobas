cmake_minimum_required(VERSION 3.10)
project(rviz2_python_binding)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Python-Documentation.html
find_package(pybind11 REQUIRED)
# find_package(python_qt_binding REQUIRED)
# find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rviz_common REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)

# src以下の全てのCPPファイルを取得
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)

# CPPファイルをビルドして共有ライブラリを作成 (インクルードは後だからここでは登録のみ？)
# cf. https://pybind11.readthedocs.io/en/stable/cmake/index.html#classic-mode
add_library(rviz2 MODULE ${LIB_CPP_FILES})

# ament_cmakeでないパッケージのインクルードディレクトリの追加
target_include_directories(rviz2 PRIVATE ${Python3_INCLUDE_DIRS})

# find_packageで追加したament_cmakeパッケージのインクルードパスの追加とリンク
ament_target_dependencies(rviz2 rviz_common)

# rviz2_python_binding/rviz2_python_bindingをPythonパッケージとしてインストール
ament_python_install_package(${PROJECT_NAME})

# 作成した共有ライブラリをPythonパッケージのパスにインストール
install(
	TARGETS rviz2
	DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
