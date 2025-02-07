cmake_minimum_required(VERSION 3.10)
project(tobas_rosbag_recorder)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tobas_linux REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_command_msgs_adapter REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)
find_package(tobas_debug_msgs_adapter REQUIRED)

set(AMENT_DEPENDENCIES
	rosbag2_cpp
	std_srvs
	tobas_linux
	tobas_node
	tobas_msgs_adapter
	tobas_command_msgs_adapter
	tobas_drone_msgs_adapter
	tobas_debug_msgs_adapter
)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "ROSBagRecorderNode" EXECUTABLE rosbag_recorder)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_package()
