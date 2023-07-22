cmake_minimum_required(VERSION 3.0.2)
project(state_estimation_eskf)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_common_actions
	tobas_tools
	dh_kdl
	dh_ros_tools
	dh_eigen_tools
	dh_std_tools
	tobas_msgs
	eigen_conversions
	kdl_msgs
	orocos_kdl
	roscpp
	actionlib
	dynamic_reconfigure
	std_msgs
)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(cfg/StateEstimationEskf.cfg)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS})

# ライブラリを作成
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

# ノードの実行ファイルを作成
file(GLOB_RECURSE NODE_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/nodes/[^.]*.cpp)
foreach(FILE IN LISTS NODE_FILES)
	get_filename_component(NODE_NAME ${FILE} NAME_WE)
	add_executable(${NODE_NAME} ${FILE})
	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
endforeach()
