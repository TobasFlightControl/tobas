cmake_minimum_required(VERSION 3.31)
project(tobas_rc_teleop)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    magic_enum
    tobas_trajectory_generation
    tobas_path_tools
    tobas_ros2_tools
    tobas_node
    tobas_msgs_adapter
    tobas_command_msgs_adapter
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME}
  magic_enum::magic_enum
  tobas_trajectory_generation::tobas_trajectory_generation
  tobas_path_tools::tobas_path_tools
  tobas_ros2_tools::tobas_ros2_tools
  tobas_node::tobas_node
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_command_msgs_adapter::tobas_command_msgs_adapter
)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} magic_enum::magic_enum)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::rc::RCTeleopNode" EXECUTABLE rc_teleop)
tobas_target_defaults(rc_teleop)

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
