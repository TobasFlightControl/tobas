cmake_minimum_required(VERSION 3.10)
project(tobas_a1_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_tools
	tobas_a1_core
	tobas_hal_core
	tobas_hal_msgs
	nodelet
	pluginlib
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
	CATKIN_DEPENDS
		tobas_tools
		tobas_a1_core
		tobas_hal_core
		tobas_hal_msgs
		nodelet
		pluginlib
)

include_directories(${catkin_INCLUDE_DIRS} include)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} stdc++fs)

file(GLOB_RECURSE NODELET_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodelets/[^.]*.cpp)
add_library(${PROJECT_NAME}_nodelet SHARED ${NODELET_FILES})
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME} stdc++fs)

file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
set(targets_to_install "")
foreach(FILE IN LISTS EXE_FILES)
	get_filename_component(NAME ${FILE} NAME_WE)
	add_executable(${NAME} ${FILE})
	target_link_libraries(${NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
	list(APPEND targets_to_install ${NAME})
endforeach()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${targets_to_install}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
