cmake_minimum_required(VERSION 3.25)
project(tobas_object_avoidance)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tobas_constants REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_tools REQUIRED)
find_package(tobas_drone_tools REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)
find_package(tobas_debug_msgs_adapter REQUIRED)

include_directories(include)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(tobas_object_avoidance src/tobas_object_avoidance.cpp)
target_include_directories(
  tobas_object_avoidance PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

target_link_libraries(tobas_object_avoidance ${OCTOMAP_LIBRARIES})

ament_target_dependencies(
  tobas_object_avoidance
  rclcpp
  geometry_msgs
  octomap_msgs
  tobas_msgs
  tobas_constants
  tobas_node
  tobas_tools
  tobas_drone_tools
  tobas_drone_msgs_adapter
  tobas_debug_msgs_adapter
)

install(TARGETS tobas_object_avoidance DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
