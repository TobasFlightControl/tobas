cmake_minimum_required(VERSION 3.10)
project(tobas_components_rt)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tobas_linux REQUIRED)

set(AMENT_DEPENDENCIES
	rclcpp
	rclcpp_components
	tobas_linux
)

include_directories(include)

file(GLOB_RECURSE NODE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
set(targets_to_install "")

foreach(FILE IN LISTS NODE_FILES)
	get_filename_component(NAME ${FILE} NAME_WE)
	add_executable(${NAME} ${FILE})
	ament_target_dependencies(${NAME} ${AMENT_DEPENDENCIES})
	list(APPEND targets_to_install ${NAME})
endforeach()

install(TARGETS ${targets_to_install} DESTINATION lib/${PROJECT_NAME})

ament_package()
