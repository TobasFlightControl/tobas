cmake_minimum_required(VERSION 3.25)
project(tobas_gazebo_gui_plugins)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES tobas_gazebo_common tobas_gazebo_conversions tobas_gazebo_tools)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

# Build common library.
file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

# Build each plugin.
file(GLOB PLUGIN_DIR_PATHS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} plugins/*)
set(TARGETS_TO_INSTALL "")
foreach(dir_path IN LISTS PLUGIN_DIR_PATHS)
  get_filename_component(PLUGIN_NAME ${dir_path} NAME_WE)
  qt5_wrap_cpp(${PLUGIN_NAME}_MOC ${dir_path}/${PLUGIN_NAME}.hpp) # Variable names must be unique
  qt_add_resources(${PLUGIN_NAME}_RCC ${dir_path}/${PLUGIN_NAME}.qrc) # Variable names must be unique
  add_library(${PLUGIN_NAME} SHARED ${${PLUGIN_NAME}_MOC} ${${PLUGIN_NAME}_RCC} ${dir_path}/${PLUGIN_NAME}.cpp)
  target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME} gz-sim::gz-sim gz-plugin::register)
  target_link_options(${PLUGIN_NAME} PRIVATE -Wl,--no-undefined)
  list(APPEND TARGETS_TO_INSTALL ${PLUGIN_NAME})
endforeach()

# Install plugins.
install(TARGETS ${TARGETS_TO_INSTALL} EXPORT ${PROJECT_NAME} LIBRARY DESTINATION lib/${PROJECT_NAME}/plugins)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")

# Export plugins.
ament_export_libraries(${TARGETS_TO_INSTALL})

ament_package()
