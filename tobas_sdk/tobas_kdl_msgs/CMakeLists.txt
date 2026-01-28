cmake_minimum_required(VERSION 3.25)
project(tobas_kdl_msgs)

set(AMENT_DEPENDENCIES std_msgs tobas_eigen_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE IFACE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg srv/*.srv action/*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${IFACE_TARGETS} DEPENDENCIES ${AMENT_DEPENDENCIES})

ament_export_dependencies(rosidl_default_runtime ${AMENT_DEPENDENCIES})

ament_package()
