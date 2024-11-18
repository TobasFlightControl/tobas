cmake_minimum_required(VERSION 3.10)
project(tobas_ros_interface)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(tobas_drone_msgs REQUIRED)
find_package(tobas_real_common REQUIRED)
find_package(tobas_real_msgs REQUIRED)

set(AMENT_DEPENDENCIES
	std_srvs
	tobas_node
	tobas_msgs
	tobas_drone_msgs
	tobas_real_common
	tobas_real_msgs
)

include_directories(include)

add_executable(ros_interface nodes/ros_interface.cpp)
ament_target_dependencies(ros_interface ${AMENT_DEPENDENCIES})

install(TARGETS ros_interface DESTINATION lib/${PROJECT_NAME})

ament_package()
