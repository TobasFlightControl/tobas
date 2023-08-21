cmake_minimum_required(VERSION 3.0.2)
project(tobas_tools)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	dh_kdl
	dh_ros_tools
	dh_eigen_tools
	dh_std_tools
	kdl_parser
	tobas_msgs
)

catkin_package(
	INCLUDE_DIRS include XYZgeomag/src
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS dh_kdl dh_ros_tools dh_eigen_tools dh_std_tools kdl_parser tobas_msgs
)

include_directories(${catkin_INCLUDE_DIRS} XYZgeomag/src)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
