cmake_minimum_required(VERSION 3.10)
project(tobas_property_server)

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
find_package(tobas_path_tools REQUIRED)
find_package(tobas_linux REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_property_tree REQUIRED)
find_package(tobas_property_common REQUIRED)
find_package(tobas_property_msgs REQUIRED)

set(AMENT_DEPENDENCIES
	std_srvs
	tobas_path_tools
	tobas_linux
	tobas_node
	tobas_property_tree
	tobas_property_common
	tobas_property_msgs
)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "ptree::PropertyServer" EXECUTABLE property_server)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
