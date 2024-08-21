cmake_minimum_required(VERSION 3.10)
project(state_estimation_eskf)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(tobas_eigen_tools REQUIRED)
find_package(tobas_geomag REQUIRED)
find_package(tobas_kdl_conversions REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(tobas_debug_msgs REQUIRED)

include_directories(include)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}
	tobas_eigen_tools
	tobas_geomag
	tobas_kdl_conversions
	tobas_node
	tobas_msgs
	tobas_debug_msgs
)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "ObserverNode" EXECUTABLE observer)

install(
	TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
	tobas_eigen_tools
	tobas_geomag
	tobas_kdl_conversions
	tobas_node
	tobas_msgs
	tobas_debug_msgs
)

ament_package()
