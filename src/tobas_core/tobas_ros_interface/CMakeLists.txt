cmake_minimum_required(VERSION 3.31)
project(tobas_ros_interface)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    std_srvs
    tobas_node
    tobas_tools
    tobas_kdl_msgs
    tobas_msgs
    tobas_drone_msgs
    tobas_real_common
    tobas_real_msgs
    tobas_mission_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_executable(ros_interface ${NODE_TARGETS})
tobas_target_defaults(ros_interface)
target_link_libraries(
  ros_interface
  std_srvs::std_srvs
  tobas_node::tobas_node
  tobas_tools::tobas_tools
  tobas_kdl_msgs::tobas_kdl_msgs
  tobas_msgs::tobas_msgs
  tobas_drone_msgs::tobas_drone_msgs
  tobas_real_common::tobas_real_common
  tobas_real_msgs::tobas_real_msgs
  tobas_mission_msgs::tobas_mission_msgs
)

install(TARGETS ros_interface DESTINATION lib/${PROJECT_NAME})

ament_package()
