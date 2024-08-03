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
# find_package(ament_cmake_python REQUIRED)
# find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rviz_common REQUIRED)
# find_package(python_qt_binding REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(rviz2 SHARED ${LIB_CPP_FILES})
target_include_directories(rviz2 PRIVATE ${Python3_INCLUDE_DIRS})
ament_target_dependencies(rviz2 rviz_common pybind11 Python3)

install(TARGETS rviz2 DESTINATION lib/${PROJECT_NAME})

ament_package()
