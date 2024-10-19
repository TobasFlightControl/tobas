cmake_minimum_required(VERSION 3.10)
project(tobas_dparam_server)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_dparam_common REQUIRED)
find_package(tobas_dparam_msgs REQUIRED)

set(AMENT_DEPENDENCIES
	tobas_node
	tobas_dparam_common
	tobas_dparam_msgs
)

include_directories(include)

add_executable(dparam_server nodes/dparam_server.cpp)
ament_target_dependencies(dparam_server ${AMENT_DEPENDENCIES})

install(TARGETS dparam_server DESTINATION lib/${PROJECT_NAME})

ament_package()
