cmake_minimum_required(VERSION 3.10)
project(tobas_real_common)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
