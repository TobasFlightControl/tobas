cmake_minimum_required(VERSION 3.25)
project(tobas_drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tobas_std_msgs REQUIRED)
find_package(tobas_kdl_msgs REQUIRED)

set(AMENT_DEPENDENCIES tobas_std_msgs tobas_kdl_msgs)

file(GLOB_RECURSE MSG_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg srv/[^.]*.srv action/[^.]*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES ${AMENT_DEPENDENCIES})

ament_export_dependencies(rosidl_default_runtime ${AMENT_DEPENDENCIES})

ament_package()
