cmake_minimum_required(VERSION 3.10)
project(tobas_kdl_msgs_adapter)

find_package(ament_cmake REQUIRED)
find_package(tobas_kdl REQUIRED)
find_package(tobas_kdl_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
ament_target_dependencies(${PROJECT_NAME} INTERFACE tobas_kdl tobas_kdl_msgs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(tobas_kdl tobas_kdl_msgs)

ament_package()
