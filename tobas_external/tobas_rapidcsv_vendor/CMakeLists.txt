cmake_minimum_required(VERSION 3.25)
project(tobas_rapidcsv_vendor)

find_package(ament_cmake REQUIRED)

add_library(rapidcsv INTERFACE)
target_include_directories(
  rapidcsv INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/rapidcsv> $<INSTALL_INTERFACE:include>
)

install(
  TARGETS rapidcsv
  EXPORT export_${PROJECT_NAME}
  INCLUDES
  DESTINATION include
)
install(FILES vendor/rapidcsv/rapidcsv.h DESTINATION include)
install(FILES vendor/rapidcsv/LICENSE README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})

ament_package()
