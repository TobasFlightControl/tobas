cmake_minimum_required(VERSION 3.10)
project(tobas_real_ros)

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
find_package(tobas_linux REQUIRED)
find_package(tobas_property_tree REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_real_common REQUIRED)
find_package(tobas_real_msgs REQUIRED)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node
	sensor_msgs
	tobas_linux
	tobas_property_tree
	tobas_node
	tobas_msgs_adapter
	tobas_real_common
	tobas_real_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "ImuHandlerNode" EXECUTABLE imu_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "MagnetometerHandlerNode" EXECUTABLE magnetometer_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "BarometerHandlerNode" EXECUTABLE barometer_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "BatteryHandlerNode" EXECUTABLE battery_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "RCInputHandlerNode" EXECUTABLE rcin_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "CpuHandlerNode" EXECUTABLE cpu_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "TimeReferenceServerNode" EXECUTABLE time_reference_server)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_package()
