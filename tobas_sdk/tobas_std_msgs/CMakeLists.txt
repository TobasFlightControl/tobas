cmake_minimum_required(VERSION 3.25)
project(tobas_std_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(AMENT_DEPENDENCIES std_msgs)

file(GLOB_RECURSE IFACE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg srv/*.srv action/*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${IFACE_TARGETS} DEPENDENCIES ${AMENT_DEPENDENCIES})

ament_export_dependencies(rosidl_default_runtime ${AMENT_DEPENDENCIES})

ament_package()
