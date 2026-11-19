cmake_minimum_required(VERSION 3.31)
project(tobas_gazebo_msgs)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES geometry_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE IFACE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg srv/*.srv action/*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${IFACE_TARGETS} DEPENDENCIES ${AMENT_DEPENDENCIES})

# rosidl creates its compiled targets internally, alongside interface and utility targets.
get_property(rosidl_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(rosidl_target IN LISTS rosidl_targets)
  get_target_property(target_type ${rosidl_target} TYPE)
  if(target_type MATCHES "^(STATIC|SHARED|MODULE|OBJECT)_LIBRARY$")
    tobas_target_defaults(${rosidl_target})
  endif()
endforeach()

ament_export_dependencies(rosidl_default_runtime ${AMENT_DEPENDENCIES})

ament_package()
