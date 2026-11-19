cmake_minimum_required(VERSION 3.31)
project(tobas_kdl)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_math tobas_std_tools tobas_eigen_tools tobas_quadprog)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME} tobas_math::tobas_math tobas_std_tools::tobas_std_tools tobas_eigen_tools::tobas_eigen_tools
  tobas_quadprog::tobas_quadprog
)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)

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
ament_export_dependencies(${AMENT_DEPENDENCIES})

# Include license texts and third-party notices to satisfy redistribution requirements.
install(FILES ../../../../LICENSE ../../../../THIRD_PARTY_NOTICES.txt DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../../../LICENSES DESTINATION share/${PROJECT_NAME})

ament_package()
