cmake_minimum_required(VERSION 3.10)
project(tobas_keyboard_teleop)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_multirotor_takeoff
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS tobas_multirotor_takeoff
)

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
set(targets_to_install "")
foreach(FILE IN LISTS EXE_FILES)
	get_filename_component(NODE_NAME ${FILE} NAME_WE)
	add_executable(${NODE_NAME} ${FILE})
	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
	list(APPEND targets_to_install ${NODE_NAME})
endforeach()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${targets_to_install}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
