cmake_minimum_required(VERSION 3.25)
project(tobas_rviz_plugin)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES tf2_eigen rviz_common rviz_default_plugins tobas_visualization_msgs)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME}_lib STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME}_lib ${AMENT_DEPENDENCIES})

file(GLOB_RECURSE PLUGIN_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} plugins/*.cpp)
add_library(${PROJECT_NAME} SHARED ${PLUGIN_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)

ament_package()
