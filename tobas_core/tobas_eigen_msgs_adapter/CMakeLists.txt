cmake_minimum_required(VERSION 3.10)
project(tobas_eigen_msgs_adapter)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tobas_eigen_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
ament_target_dependencies(${PROJECT_NAME} INTERFACE Eigen3 tobas_eigen_msgs)

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
ament_export_dependencies(Eigen3 tobas_eigen_msgs)

ament_package()
