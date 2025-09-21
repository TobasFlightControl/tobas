cmake_minimum_required(VERSION 3.25)
project(tobas_gazebo_system_plugins)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES
    std_srvs
    tobas_dsp
    tobas_geomag
    tobas_nlp
    tobas_node
    tobas_tools
    tobas_wind_model
    tobas_msgs_adapter
    tobas_drone_tools
    tobas_gazebo_common
    tobas_gazebo_conversions
    tobas_gazebo_tools
    tobas_gazebo_msgs
)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include ${OpenCV_INCLUDE_DIRS})

# Build common library
file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} gz-sim::gz-sim gz-plugin::register)

# Build each plugin
file(GLOB_RECURSE PLUGIN_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} plugins/*.cpp)
set(TARGETS_TO_INSTALL "")
foreach(file_path IN LISTS PLUGIN_TARGETS)
  get_filename_component(FILE_NAME ${file_path} NAME_WE)
  set(PLUGIN_NAME tobas_gazebo_${FILE_NAME})
  add_library(${PLUGIN_NAME} SHARED ${file_path})
  target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME})
  target_link_options(${PLUGIN_NAME} PRIVATE -Wl,--no-undefined)
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
