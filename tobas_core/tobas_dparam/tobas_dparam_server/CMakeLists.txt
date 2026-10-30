cmake_minimum_required(VERSION 3.25)
project(tobas_dparam_server)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES tobas_node tobas_dparam_common tobas_dparam_msgs)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(dparam_server nodes/dparam_server.cpp)
target_link_libraries(
  dparam_server tobas_node::tobas_node tobas_dparam_common::tobas_dparam_common tobas_dparam_msgs::tobas_dparam_msgs
)

install(TARGETS dparam_server DESTINATION lib/${PROJECT_NAME})

ament_package()
