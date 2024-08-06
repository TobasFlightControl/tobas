cmake_minimum_required(VERSION 3.10)
project(tobas_drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tobas_drone_core)
find_package(tobas_std_msgs)
find_package(tobas_kdl_msgs)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg srv/[^.]*.srv action/[^.]*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES tobas_std_msgs tobas_kdl_msgs)

add_library(${PROJECT_NAME}_if INTERFACE)

install(
  TARGETS ${PROJECT_NAME}_if
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime tobas_drone_core tobas_std_msgs tobas_kdl_msgs)

ament_package()
