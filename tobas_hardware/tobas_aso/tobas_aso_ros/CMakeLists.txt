cmake_minimum_required(VERSION 3.10)
project(tobas_aso_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(tobas_ic_drivers REQUIRED)
find_package(tobas_property_tree REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)
find_package(tobas_real_common REQUIRED)
find_package(tobas_hardware_common REQUIRED)
find_package(tobas_aso_core REQUIRED)

set(AMENT_DEPENDENCIES
	tobas_ic_drivers
	tobas_property_tree
	tobas_msgs_adapter
	tobas_drone_msgs_adapter
	tobas_real_common
	tobas_hardware_common
	tobas_aso_core
)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
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
