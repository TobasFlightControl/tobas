cmake_minimum_required(VERSION 3.31)
project(tobas_setup_assistant)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    ament_index_cpp
    magic_enum
    sensor_msgs
    visualization_msgs
    tobas_rapidcsv_vendor
    tobas_string_tools
    tobas_xml_tools
    tobas_git
    tobas_kdl_parser
    tobas_path_tools
    tobas_ros2_tools
    tobas_tools
    tobas_urdf
    tobas_drone_msgs_adapter
    tobas_visualization_msgs
    tobas_uadf
    tobas_qt_tools
    tobas_rviz_wrapper
    tobas_gui_common
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(INJA REQUIRED IMPORTED_TARGET inja)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME}
  PkgConfig::INJA
  ament_index_cpp::ament_index_cpp
  magic_enum::magic_enum
  sensor_msgs::sensor_msgs
  visualization_msgs::visualization_msgs
  tobas_rapidcsv_vendor::rapidcsv
  tobas_string_tools::tobas_string_tools
  tobas_xml_tools::tobas_xml_tools
  tobas_git::tobas_git
  tobas_kdl_parser::tobas_kdl_parser
  tobas_path_tools::tobas_path_tools
  tobas_ros2_tools::tobas_ros2_tools
  tobas_tools::tobas_tools
  tobas_urdf::tobas_urdf
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_visualization_msgs::tobas_visualization_msgs
  tobas_uadf::tobas_uadf
  tobas_qt_tools::tobas_qt_tools
  tobas_rviz_wrapper::tobas_rviz_wrapper
  tobas_gui_common::tobas_gui_common
)

add_executable(main nodes/main.cpp)
tobas_target_defaults(main)
target_link_libraries(main ${PROJECT_NAME})
set_target_properties(main PROPERTIES OUTPUT_NAME "TobasSetupAssistant")

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config data launch resources templates DESTINATION share/${PROJECT_NAME})

ament_package()
