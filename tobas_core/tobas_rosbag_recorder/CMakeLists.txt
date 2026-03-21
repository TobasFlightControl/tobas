cmake_minimum_required(VERSION 3.25)
project(tobas_rosbag_recorder)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES
    rosbag2_cpp
    std_srvs
    tobas_path_tools
    tobas_linux
    tobas_node
    tobas_msgs_adapter
    tobas_command_msgs_adapter
    tobas_drone_msgs_adapter
    tobas_debug_msgs_adapter
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "RosbagRecorderNode" EXECUTABLE rosbag_recorder)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_package()
