cmake_minimum_required(VERSION 3.10)
project(tobas_command_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tobas_kdl_msgs REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg srv/[^.]*.srv action/[^.]*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES tobas_kdl_msgs)

ament_export_dependencies(rosidl_default_runtime tobas_kdl_msgs)

ament_package()
