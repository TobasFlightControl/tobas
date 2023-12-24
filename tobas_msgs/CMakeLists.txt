cmake_minimum_required(VERSION 3.10)
project(tobas_msgs)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_kdl_msgs
	tobas_kdl
	message_generation
	std_msgs
	sensor_msgs
	nav_msgs
	actionlib_msgs
)

# catkin_python_setup()

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/[^.]*.msg)
add_message_files(FILES ${MSG_FILES})
# file(GLOB_RECURSE SRV_FILES RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/[^.]*.srv)
# add_service_files(FILES ${SRV_FILES})
file(GLOB_RECURSE ACTION_FILES RELATIVE ${PROJECT_SOURCE_DIR}/action action/[^.]*.action)
add_action_files(FILES ${ACTION_FILES})

generate_messages(DEPENDENCIES
	tobas_kdl_msgs
	std_msgs
	sensor_msgs
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
		tobas_kdl_msgs
		tobas_kdl
		message_generation
		std_msgs
		sensor_msgs
		nav_msgs
		actionlib_msgs 
)

include_directories(${catkin_INCLUDE_DIRS} include)

# Library
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Executables
# file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
# foreach(FILE IN LISTS EXE_FILES)
# 	get_filename_component(NODE_NAME ${FILE} NAME_WE)
# 	add_executable(${NODE_NAME} ${FILE})
# 	add_dependencies(${NODE_NAME} ${PROJECT_NAME}_generate_messages_cpp)
# 	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})
# endforeach()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
