cmake_minimum_required(VERSION 3.10)
project(tobas_a1_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(tobas_kdl REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(tobas_hal_core REQUIRED)
find_package(tobas_hal_msgs REQUIRED)
find_package(tobas_a1_core REQUIRED)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node
	tobas_kdl
	tobas_msgs
	tobas_hal_core
	tobas_hal_msgs
	tobas_a1_core
)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "IMUDriverNode" EXECUTABLE imu_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "MagDriverNode" EXECUTABLE mag_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "BaroDriverNode" EXECUTABLE baro_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "GNSSDriverNode" EXECUTABLE gnss_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "ADCDriverNode" EXECUTABLE adc_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "SBUSDriverNode" EXECUTABLE sbus_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DShotDriverNode" EXECUTABLE dshot_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "PWMDriverNode" EXECUTABLE pwm_driver)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
