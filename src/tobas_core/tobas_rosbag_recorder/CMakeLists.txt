cmake_minimum_required(VERSION 3.31)
project(tobas_rosbag_recorder)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    rosbag2_cpp
    std_srvs
    tobas_path_tools
    tobas_linux
    tobas_node
    tobas_msgs_adapter
    tobas_command_msgs_adapter
    tobas_drone_msgs_adapter
    tobas_debug_msgs_adapter
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME}
  rosbag2_cpp::rosbag2_cpp
  std_srvs::std_srvs
  tobas_path_tools::tobas_path_tools
  tobas_linux::tobas_linux
  tobas_node::tobas_node
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_command_msgs_adapter::tobas_command_msgs_adapter
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_debug_msgs_adapter::tobas_debug_msgs_adapter
)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "tobas::RosbagRecorderNode" EXECUTABLE rosbag_recorder)
tobas_target_defaults(rosbag_recorder)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_package()
