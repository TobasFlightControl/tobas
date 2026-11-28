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

set(NODE_TARGETS action_interface service_interface topic_interface_logic_to_iface topic_interface_iface_to_logic)
foreach(target IN ITEMS ${NODE_TARGETS})
  add_executable(${target} nodes/${target}.cpp)
  tobas_target_defaults(${target})
  target_link_libraries(
    ${target}
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
endforeach()

install(TARGETS ${NODE_TARGETS} DESTINATION lib/${PROJECT_NAME})

ament_package()
