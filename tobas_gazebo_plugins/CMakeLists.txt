cmake_minimum_required(VERSION 3.0.2)
project(tobas_gazebo_plugins)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_tools
	gazebo_plugins
	laser_geometry
	sensor_msgs
	geometry_msgs
)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS tobas_tools gazebo_plugins laser_geometry sensor_msgs geometry_msgs
	DEPENDS GAZEBO OpenCV
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})  # RayPluginを使うために必要

# ライブラリを作成 (後でプラグインとリンクするために共有ライブラリにしておく)
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} SHARED ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# プラグインごとに共有ライブラリを作成
file(GLOB_RECURSE PLUGIN_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} plugins/[^.]*_plugin.cpp)
foreach(FILE IN LISTS PLUGIN_CPP_FILES)
	get_filename_component(FILE_NAME ${FILE} NAME_WE)
	set(PLUGIN_NAME tobas_gazebo_${FILE_NAME})
	add_library(${PLUGIN_NAME} SHARED ${FILE})
	target_link_libraries(${PLUGIN_NAME}
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES}
		${OpenCV_LIBRARIES}
		RayPlugin  # lidar_pluginに必要
		${PROJECT_NAME}
	)
endforeach()
