cmake_minimum_required(VERSION 3.10)
project(tobas_kdl_msgs)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_kdl
	message_generation
	actionlib_msgs
)

catkin_python_setup()

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/[^.]*.msg)
add_message_files(FILES ${MSG_FILES})

generate_messages(DEPENDENCIES
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS tobas_kdl message_generation actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}_cpp/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY
  src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE
)
