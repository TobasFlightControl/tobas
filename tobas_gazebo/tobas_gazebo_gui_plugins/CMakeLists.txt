cmake_minimum_required(VERSION 3.25)
project(tobas_gazebo_gui_plugins)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)

if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
  message(STATUS "This package is skipped on Raspberry Pi.")
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
find_package(tobas_gazebo_common REQUIRED)
find_package(tobas_gazebo_tools REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

# add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

# Build common library
file(GLOB_RECURSE LIB_CPP_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} tobas_gazebo_common tobas_gazebo_tools)

# Build each plugin
file(GLOB PLUGIN_DIR_PATHS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} plugins/*)
set(TARGETS_TO_INSTALL "")

foreach(dir_path IN LISTS PLUGIN_DIR_PATHS)
  get_filename_component(PLUGIN_NAME ${dir_path} NAME_WE)

  qt5_wrap_cpp(MOC_SOURCE ${dir_path}/${PLUGIN_NAME}.hpp)
  qt_add_resources(RCC_SOURCE ${dir_path}/${PLUGIN_NAME}.qrc)

  add_library(${PLUGIN_NAME} SHARED ${MOC_SOURCE} ${RCC_SOURCE} ${dir_path}/${PLUGIN_NAME}.cpp)
  target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME} gz-sim::gz-sim gz-plugin::register)

  list(APPEND TARGETS_TO_INSTALL ${PLUGIN_NAME})
endforeach()

# Install plugins
install(TARGETS ${TARGETS_TO_INSTALL} EXPORT ${PROJECT_NAME} LIBRARY DESTINATION lib/${PROJECT_NAME}/plugins)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")

# Export plugins
ament_export_libraries(${TARGETS_TO_INSTALL})

ament_package()
