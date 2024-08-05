cmake_minimum_required(VERSION 3.10)
project(tobas_drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg srv/[^.]*.srv action/[^.]*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES)

# install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
