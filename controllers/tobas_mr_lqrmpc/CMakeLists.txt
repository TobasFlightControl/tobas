cmake_minimum_required(VERSION 3.0.2)
project(tobas_mr_lqrmpc)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_mr_rotation_mpc
	tobas_mr_translation_lqr
	tobas_mr_common
	dynamic_reconfigure
	nodelet
	pluginlib
	message_generation
)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/[^.]*.msg)
add_message_files(FILES ${MSG_FILES})
# file(GLOB_RECURSE SRV_FILES RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/[^.]*.srv)
# add_service_files(FILES ${SRV_FILES})
# file(GLOB_RECURSE ACTION_FILES RELATIVE ${PROJECT_SOURCE_DIR}/action action/[^.]*.action)
# add_action_files(FILES ${ACTION_FILES})

generate_messages(DEPENDENCIES
	tobas_msgs
	kdl_msgs
	std_msgs
)

generate_dynamic_reconfigure_options(cfg/Controller.cfg)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
	CATKIN_DEPENDS
		tobas_mr_rotation_mpc
		tobas_mr_translation_lqr
		tobas_mr_common
		dynamic_reconfigure
		nodelet
		pluginlib
		message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	# /usr/local/lib/libquadprog.a
	# /usr/local/lib/libqpOASES.a
)

file(GLOB_RECURSE NODELET_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodelets/[^.]*.cpp)
add_library(${PROJECT_NAME}_nodelet SHARED ${NODELET_FILES})
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME})

file(GLOB_RECURSE NODE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
foreach(FILE IN LISTS NODE_FILES)
	get_filename_component(NODE_NAME ${FILE} NAME_WE)
	add_executable(${NODE_NAME} ${FILE})
	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
endforeach()
