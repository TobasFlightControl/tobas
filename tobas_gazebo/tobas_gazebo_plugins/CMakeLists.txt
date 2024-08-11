cmake_minimum_required(VERSION 3.10)
project(tobas_gazebo_plugins)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)

if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
	message("This package is skipped on Raspberry Pi.")
	find_package(ament_cmake REQUIRED)
	ament_package()
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

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tobas_dsp REQUIRED)
find_package(tobas_kdl REQUIRED)
find_package(tobas_path_tools REQUIRED)
find_package(tobas_ros2_tools REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(tobas_gazebo_common REQUIRED)
find_package(tobas_gazebo_tools REQUIRED)
find_package(tobas_gazebo_msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Build common library
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}
	sensor_msgs
	geometry_msgs
	std_srvs
	laser_geometry
	tobas_dsp
	tobas_kdl
	tobas_path_tools
	tobas_ros2_tools
	tobas_node
	tobas_msgs
	tobas_gazebo_common
	tobas_gazebo_tools
	tobas_gazebo_msgs
)
target_link_libraries(${PROJECT_NAME}
	gz-sim::gz-sim
	gz-plugin::register
	${OpenCV_LIBRARIES}
)

# Build each plugin
# file(GLOB_RECURSE PLUGIN_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} plugins/[^.]*.cpp)
set(PLUGIN_CPP_FILES
	plugins/barometer_plugin.cpp
	plugins/rotor_speeds_publisher_plugin.cpp
	plugins/battery_plugin.cpp
)
set(targets_to_install "")

foreach(FILE IN LISTS PLUGIN_CPP_FILES)
	get_filename_component(FILE_NAME ${FILE} NAME_WE)
	set(PLUGIN_NAME tobas_gazebo_${FILE_NAME})
	add_library(${PLUGIN_NAME} SHARED ${FILE})
	target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME})
	list(APPEND targets_to_install ${PLUGIN_NAME})
endforeach()

# Install plugins
install(
	TARGETS ${targets_to_install}
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export plugins
ament_export_libraries(${targets_to_install})

ament_package()
