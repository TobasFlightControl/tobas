cmake_minimum_required(VERSION 3.10)
project(tobas_gazebo_plugins)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)
if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
  message("This package is skipped on Raspberry Pi.")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_gazebo_common
	tobas_gazebo_msgs
	tobas_tools
	tobas_dsp
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
	CATKIN_DEPENDS
		tobas_gazebo_common
		tobas_gazebo_msgs
		tobas_tools
		tobas_dsp
		gazebo_plugins
		laser_geometry
		sensor_msgs
		geometry_msgs
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
set(targets_to_install "")
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
	list(APPEND targets_to_install ${PLUGIN_NAME})
endforeach()

# インストール (共有ライブラリは実行時にリンクされるため，全てインストールディレクトリに含める必要がある)
install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${targets_to_install}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
