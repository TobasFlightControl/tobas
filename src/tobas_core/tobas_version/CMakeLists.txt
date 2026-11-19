cmake_minimum_required(VERSION 3.31)
project(tobas_version)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} INTERFACE)
ament_generate_version_header(${PROJECT_NAME} HEADER_PATH "${PROJECT_NAME}/version.h" INSTALL_PATH "include")

file(GLOB_RECURSE EXEC_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} examples/*.cpp)
set(TARGETS_TO_INSTALL "")
foreach(file_path IN LISTS EXEC_TARGETS)
  get_filename_component(NAME ${file_path} NAME_WE)
  add_executable(${NAME} ${file_path})
  tobas_target_defaults(${NAME})
  target_link_libraries(${NAME} ${PROJECT_NAME})
  list(APPEND TARGETS_TO_INSTALL ${NAME})
endforeach()

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
install(TARGETS ${TARGETS_TO_INSTALL} DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
