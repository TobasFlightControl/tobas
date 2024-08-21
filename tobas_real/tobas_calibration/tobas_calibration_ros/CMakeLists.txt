cmake_minimum_required(VERSION 3.10)
project(tobas_calibration_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(tobas_property_tools REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)
find_package(tobas_hal_core REQUIRED)
find_package(tobas_hal_msgs_adapter REQUIRED)
find_package(tobas_real_common REQUIRED)
find_package(tobas_calibration_msgs REQUIRED)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node
	tobas_property_tools
	tobas_msgs_adapter
	tobas_drone_msgs_adapter
	tobas_hal_core
	tobas_hal_msgs_adapter
	tobas_real_common
	tobas_calibration_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "AccelCalibrationNode" EXECUTABLE accel_calibration)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "AdcCalibrationNode" EXECUTABLE adc_calibration)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "EscCalibrationNode" EXECUTABLE esc_calibration)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "MagCalibrationNode" EXECUTABLE mag_calibration)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "RCInputCalibrationNode" EXECUTABLE rcin_calibration)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_package()
