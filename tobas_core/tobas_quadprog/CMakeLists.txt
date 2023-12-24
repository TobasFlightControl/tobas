cmake_minimum_required(VERSION 3.10)
project(tobas_quadprog)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
	tobas_eigen_tools
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS tobas_eigen_tools
)

include_directories(${catkin_INCLUDE_DIRS} include)

# Library
set(LIB_CPP_FILES
	src/qpsolver.cpp
	src/dual_active_set.cpp
	src/primal_dual_interior_point.cpp
	# src/quadprogpp.cpp
	# src/qpoases.cpp
)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Executives
# file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} examples/[^.]*.cpp)
# foreach(FILE IN LISTS EXE_FILES)
# 	get_filename_component(NAME ${FILE} NAME_WE)
# 	add_executable(${NAME} ${FILE})
# 	target_link_libraries(${NAME}
# 		${catkin_LIBRARIES}
# 		${PROJECT_NAME}
# 		/usr/local/lib/libquadprog.a
# 		/usr/local/lib/libqpOASES.a
# 	)
# endforeach()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
