cmake_minimum_required(VERSION 3.31)
project(tobas_real_ros)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    magic_enum
    sensor_msgs
    tobas_string_tools
    tobas_dsp
    tobas_linux
    tobas_property_tree
    tobas_node
    tobas_tools
    tobas_msgs_adapter
    tobas_drone_msgs_adapter
    tobas_real_common
    tobas_real_msgs
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
  magic_enum::magic_enum
  sensor_msgs::sensor_msgs
  tobas_string_tools::tobas_string_tools
  tobas_dsp::tobas_dsp
  tobas_linux::tobas_linux
  tobas_property_tree::tobas_property_tree
  tobas_node::tobas_node
  tobas_tools::tobas_tools
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_real_common::tobas_real_common
  tobas_real_msgs::tobas_real_msgs
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::real::ImuHandlerNode" EXECUTABLE imu_handler)
tobas_target_defaults(imu_handler)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::real::MagnetometerHandlerNode" EXECUTABLE magnetometer_handler
)
tobas_target_defaults(magnetometer_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::real::RCInputHandlerNode" EXECUTABLE rcin_handler)
tobas_target_defaults(rcin_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::real::CpuHandlerNode" EXECUTABLE cpu_handler)
tobas_target_defaults(cpu_handler)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::real::JointsHandlerNode" EXECUTABLE joints_handler)
tobas_target_defaults(joints_handler)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::real::IcePropulsionSystemHandlerNode" EXECUTABLE ice_propulsion_system_handler
)
tobas_target_defaults(ice_propulsion_system_handler)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_package()
