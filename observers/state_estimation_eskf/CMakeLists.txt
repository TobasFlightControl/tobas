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
	eigen_conversions
	dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/StateEstimationEskf.cfg)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS tobas_common_actions eigen_conversions dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS})

# ライブラリを作成
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# ノードの実行ファイルを作成
file(GLOB_RECURSE NODE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
foreach(FILE IN LISTS NODE_FILES)
	get_filename_component(NODE_NAME ${FILE} NAME_WE)
	add_executable(${NODE_NAME} ${FILE})
	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
endforeach()
