cmake_minimum_required(VERSION 3.10)
project(ardupilot_dynamic_configuration)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

# file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/[^.]*.msg)
# add_message_files(FILES ${MSG_FILES})
# file(GLOB_RECURSE SRV_FILES RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/[^.]*.srv)
# add_service_files(FILES ${SRV_FILES})
# file(GLOB_RECURSE ACTION_FILES RELATIVE ${PROJECT_SOURCE_DIR}/action action/[^.]*.action)
# add_action_files(FILES ${ACTION_FILES})

# generate_messages(DEPENDENCIES
# 	actionlib_msgs
# )

# generate_dynamic_reconfigure_options(cfg/RenameMe.cfg)

catkin_package(
# 	INCLUDE_DIRS include
# 	LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
# 	CATKIN_DEPENDS
# 	DEPENDS
)

# include_directories(${catkin_INCLUDE_DIRS} include)

# file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
# add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# file(GLOB_RECURSE NODELET_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodelets/[^.]*.cpp)
# add_library(${PROJECT_NAME}_nodelet SHARED ${NODELET_FILES})
# target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME})

# file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
# foreach(FILE IN LISTS EXE_FILES)
# 	get_filename_component(NODE_NAME ${FILE} NAME_WE)
# 	add_executable(${NODE_NAME} ${FILE})
# 	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
# endforeach()

# Install
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE
)

file(GLOB_RECURSE SCRIPTS RELATIVE ${PROJECT_SOURCE_DIR} scripts/[^.]*)
catkin_install_python(PROGRAMS ${SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
