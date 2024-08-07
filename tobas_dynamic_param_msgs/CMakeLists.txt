cmake_minimum_required(VERSION 3.10)
project(tobas_dynamic_param_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg srv/[^.]*.srv action/[^.]*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)

ament_package()
