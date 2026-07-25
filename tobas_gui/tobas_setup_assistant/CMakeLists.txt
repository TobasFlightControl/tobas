cmake_minimum_required(VERSION 3.25)
project(tobas_setup_assistant)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES
    magic_enum
    sensor_msgs
    visualization_msgs
    inja_ament
    rapidcsv_ament
    tobas_string_tools
    tobas_xml_tools
    tobas_git
    tobas_kdl_parser
    tobas_path_tools
    tobas_ros2_tools
    tobas_urdf
    tobas_property_client
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

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} magic_enum::magic_enum)

add_executable(main nodes/main.cpp)
target_link_libraries(main ${PROJECT_NAME})
set_target_properties(main PROPERTIES OUTPUT_NAME "TobasSetupAssistant")

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config data launch resources templates DESTINATION share/${PROJECT_NAME})

ament_package()
