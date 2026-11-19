cmake_minimum_required(VERSION 3.31)
project(tobas_fc1xx_ros)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

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
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(
  ${PROJECT_NAME}_node
  tobas_dsp::tobas_dsp
  tobas_ic_drivers::tobas_ic_drivers
  tobas_tools::tobas_tools
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_real_common::tobas_real_common
  tobas_hardware_common::tobas_hardware_common
  tobas_fc1xx_core::tobas_fc1xx_core
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::ImuDriverNode" EXECUTABLE imu_driver)
tobas_target_defaults(imu_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::MagDriverNode" EXECUTABLE mag_driver)
tobas_target_defaults(mag_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::BaroDriverNode" EXECUTABLE baro_driver)
tobas_target_defaults(baro_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::GnssDriverNode" EXECUTABLE gnss_driver)
tobas_target_defaults(gnss_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::BatteryDriverNode" EXECUTABLE battery_driver)
tobas_target_defaults(battery_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::DShotDriverNode" EXECUTABLE dshot_driver)
tobas_target_defaults(dshot_driver)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::fc1xx::PwmDriverNode" EXECUTABLE pwm_driver)
tobas_target_defaults(pwm_driver)

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
