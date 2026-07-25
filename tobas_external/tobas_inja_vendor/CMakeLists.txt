cmake_minimum_required(VERSION 3.25)
project(tobas_inja_vendor)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(inja INTERFACE)
target_include_directories(
  inja INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/inja/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(inja INTERFACE nlohmann_json::nlohmann_json)

install(
  TARGETS inja
  EXPORT export_${PROJECT_NAME}
  INCLUDES
  DESTINATION include
)
install(DIRECTORY vendor/inja/include/inja DESTINATION include)
install(FILES vendor/inja/LICENSE README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(nlohmann_json)

ament_package()
