cmake_minimum_required(VERSION 3.10)
project(tobas_gazebo_system_plugins)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)

if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
	message("This package is skipped on Raspberry Pi.")
	find_package(ament_cmake REQUIRED)
	ament_package()
	return()
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tobas_geomag REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_wind_model REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_drone_tools REQUIRED)
find_package(tobas_gazebo_common REQUIRED)
find_package(tobas_gazebo_tools REQUIRED)
find_package(tobas_gazebo_msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Build common library
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}
	geometry_msgs
	std_srvs
	laser_geometry
	tobas_geomag
	tobas_node
	tobas_wind_model
	tobas_msgs_adapter
	tobas_drone_tools
	tobas_gazebo_common
	tobas_gazebo_tools
	tobas_gazebo_msgs
)
target_link_libraries(${PROJECT_NAME}
	gz-sim::gz-sim
	gz-plugin::register
)

# Build each plugin
file(GLOB_RECURSE PLUGIN_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} plugins/[^.]*.cpp)
set(TARGETS_TO_INSTALL "")

foreach(FILE IN LISTS PLUGIN_CPP_FILES)
	get_filename_component(FILE_NAME ${FILE} NAME_WE)
	set(PLUGIN_NAME tobas_gazebo_${FILE_NAME})
	add_library(${PLUGIN_NAME} SHARED ${FILE})
	target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME})
	list(APPEND TARGETS_TO_INSTALL ${PLUGIN_NAME})
endforeach()

# Install plugins
install(
	TARGETS ${TARGETS_TO_INSTALL}
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export plugins
ament_export_libraries(${TARGETS_TO_INSTALL})

ament_package()
