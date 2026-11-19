cmake_minimum_required(VERSION 3.31)
project(tobas_dparam_server)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_node tobas_dparam_common tobas_dparam_msgs)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(dparam_server nodes/dparam_server.cpp)
tobas_target_defaults(dparam_server)
target_link_libraries(
  dparam_server tobas_node::tobas_node tobas_dparam_common::tobas_dparam_common tobas_dparam_msgs::tobas_dparam_msgs
)

install(TARGETS dparam_server DESTINATION lib/${PROJECT_NAME})

ament_package()
