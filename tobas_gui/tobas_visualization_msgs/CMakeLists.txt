cmake_minimum_required(VERSION 3.25)
project(tobas_visualization_msgs)

set(AMENT_DEPENDENCIES std_msgs sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE IFACE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg srv/*.srv action/*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${IFACE_TARGETS} DEPENDENCIES ${AMENT_DEPENDENCIES})

ament_export_dependencies(rosidl_default_runtime ${AMENT_DEPENDENCIES})

# Include license texts and third-party notices to satisfy redistribution requirements.
install(FILES ../../LICENSE ../../THIRD_PARTY_NOTICES.txt DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../LICENSES DESTINATION share/${PROJECT_NAME})

ament_package()
