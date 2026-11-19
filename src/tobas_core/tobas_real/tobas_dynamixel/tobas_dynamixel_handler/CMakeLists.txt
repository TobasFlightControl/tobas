cmake_minimum_required(VERSION 3.31)
project(tobas_dynamixel_handler)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES std_srvs dynamixel_sdk tobas_node tobas_msgs_adapter tobas_dynamixel_msgs
                       tobas_dynamixel_ros_interface
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(
  ${PROJECT_NAME}_node
  std_srvs::std_srvs
  dynamixel_sdk::dynamixel_sdk
  tobas_node::tobas_node
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_dynamixel_msgs::tobas_dynamixel_msgs
  tobas_dynamixel_ros_interface::tobas_dynamixel_ros_interface
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::dxl::DynamixelHandlerNode" EXECUTABLE dynamixel_handler
)
tobas_target_defaults(dynamixel_handler)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::dxl::DynamixelBridgeNode" EXECUTABLE dynamixel_bridge
)
tobas_target_defaults(dynamixel_bridge)

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
