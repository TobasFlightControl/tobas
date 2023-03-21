cmake_minimum_required(VERSION 3.0.2)
project(multirotor_gazebo_plugins)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
	dh_std_tools
	multirotor_msgs
	sensor_msgs
	geometry_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# プラグインごとに共有ライブラリを作成
file(GLOB_RECURSE PLUGIN_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/[^.]*_plugin.cpp)
foreach(FILE IN LISTS PLUGIN_CPP_FILES)
	get_filename_component(FILE_NAME ${FILE} NAME_WE)
	set(PLUGIN_NAME dh_gazebo_${FILE_NAME})
	add_library(${PLUGIN_NAME} SHARED ${FILE})
	target_link_libraries(${PLUGIN_NAME} ${catkin_LIBRARIES})
endforeach()
