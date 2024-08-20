cmake_minimum_required(VERSION 3.10)
project(tobas_keyboard_teleop)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)

if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
  message("This package is skipped on Raspberry Pi.")
  find_package(ament_cmake REQUIRED)
  ament_package()
  return()
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tobas_keyboard REQUIRED)
find_package(tobas_ros2_tools REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_constants REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(tobas_drone_tools REQUIRED)
find_package(tobas_drone_msgs REQUIRED)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
set(targets_to_install "")

foreach(FILE IN LISTS NODE_CPP_FILES)
  get_filename_component(NAME ${FILE} NAME_WE)
  add_executable(${NAME} ${FILE})
  ament_target_dependencies(${NAME}
    sensor_msgs
    tobas_keyboard
    tobas_ros2_tools
    tobas_node
    tobas_constants
    tobas_msgs
    tobas_drone_tools
    tobas_drone_msgs
  )
  list(APPEND targets_to_install ${NAME})
endforeach()

install(TARGETS ${targets_to_install} DESTINATION lib/${PROJECT_NAME})

ament_package()
