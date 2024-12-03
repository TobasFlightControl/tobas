cmake_minimum_required(VERSION 3.10)
project(tobas_dynamixel_handler)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_dynamixel_msgs REQUIRED)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node
	std_srvs
	dynamixel_sdk
	tobas_node
	tobas_msgs_adapter
	tobas_dynamixel_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DynamixelHandlerNode" EXECUTABLE dynamixel_handler)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
