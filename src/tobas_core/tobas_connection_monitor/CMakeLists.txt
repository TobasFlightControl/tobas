cmake_minimum_required(VERSION 3.31)
project(tobas_connection_monitor)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_node tobas_constants tobas_msgs)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(
  ${PROJECT_NAME}_node tobas_node::tobas_node tobas_constants::tobas_constants tobas_msgs::tobas_msgs
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::HeartbeatSenderNode" EXECUTABLE heartbeat_sender)
tobas_target_defaults(heartbeat_sender)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::HeartbeatReceiverNode" EXECUTABLE heartbeat_receiver
)
tobas_target_defaults(heartbeat_receiver)

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
