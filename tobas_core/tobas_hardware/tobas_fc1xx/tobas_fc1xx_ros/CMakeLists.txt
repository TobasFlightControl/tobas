cmake_minimum_required(VERSION 3.25)
project(tobas_fc1xx_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES
    tobas_dsp
    tobas_ic_drivers
    tobas_tools
    tobas_msgs_adapter
    tobas_drone_msgs_adapter
    tobas_real_common
    tobas_hardware_common
    tobas_fc1xx_core
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::ImuDriverNode" EXECUTABLE imu_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::MagDriverNode" EXECUTABLE mag_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::BaroDriverNode" EXECUTABLE baro_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::GnssDriverNode" EXECUTABLE gnss_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::BatteryDriverNode" EXECUTABLE battery_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::DShotDriverNode" EXECUTABLE dshot_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::PwmDriverNode" EXECUTABLE pwm_driver)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
