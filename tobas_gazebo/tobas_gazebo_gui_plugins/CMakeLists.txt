cmake_minimum_required(VERSION 3.10)
project(tobas_gazebo_gui_plugins)

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
find_package(tobas_gazebo_tools REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

# add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

# Build common library
file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}
	tobas_gazebo_tools
)

# Build each plugin
file(GLOB_RECURSE PLUGIN_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} plugins/[^.]*.cpp)
set(targets_to_install "")

foreach(FILE IN LISTS PLUGIN_CPP_FILES)
	get_filename_component(PARENT_DIR ${FILE} DIRECTORY)
	get_filename_component(FILE_NAME ${FILE} NAME_WE)
	set(FILE_WITHOUT_EXT ${PARENT_DIR}/${FILE_NAME})
	set(PLUGIN_NAME tobas_gazebo_${FILE_NAME})

	qt5_wrap_cpp(MOC_SOURCE ${FILE_WITHOUT_EXT}.hpp)
	add_library(${PLUGIN_NAME} SHARED ${MOC_SOURCE} ${FILE_WITHOUT_EXT}.cpp)
	target_link_libraries(${PLUGIN_NAME}
		${PROJECT_NAME}
		gz-sim::gz-sim
		gz-plugin::register
	)

	list(APPEND targets_to_install ${PLUGIN_NAME})
endforeach()

# Install plugins
install(
	TARGETS ${targets_to_install}
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib/${PROJECT_NAME}/plugins
)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")

# Export plugins
ament_export_libraries(${targets_to_install})

ament_package()
