cmake_minimum_required(VERSION 3.31)
project(tobas_gazebo_ros)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_node tobas_tools tobas_drone_msgs_adapter tobas_msgs_adapter tobas_gazebo_common
                       tobas_gazebo_msgs
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
  tobas_node::tobas_node
  tobas_tools::tobas_tools
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_gazebo_common::tobas_gazebo_common
  tobas_gazebo_msgs::tobas_gazebo_msgs
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::gazebo::CpuHandlerNode" EXECUTABLE cpu_handler)
tobas_target_defaults(cpu_handler)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::gazebo::RotorStatesPublisherNode" EXECUTABLE rotor_states_publisher
)
tobas_target_defaults(rotor_states_publisher)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::gazebo::ElectricRotorCommandHandlerNode" EXECUTABLE
  electric_rotor_command_handler
)
tobas_target_defaults(electric_rotor_command_handler)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::gazebo::JointCommandHandlerNode" EXECUTABLE joint_command_handler
)
tobas_target_defaults(joint_command_handler)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
