cmake_minimum_required(VERSION 3.31)
project(tobas_urdf_builder)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_qt_tools tobas_rviz_wrapper tobas_gui_common)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME} tobas_qt_tools::tobas_qt_tools tobas_rviz_wrapper::tobas_rviz_wrapper
  tobas_gui_common::tobas_gui_common
)

add_executable(main nodes/main.cpp)
tobas_target_defaults(main)
target_link_libraries(main ${PROJECT_NAME})
set_target_properties(main PROPERTIES OUTPUT_NAME "TobasURDFBuilder")

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch resources DESTINATION share/${PROJECT_NAME})

ament_package()
