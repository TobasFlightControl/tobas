cmake_minimum_required(VERSION 3.10)
project(magic_enum_ament)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
ament_target_dependencies(${PROJECT_NAME} INTERFACE)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/magic_enum/include/magic_enum DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
