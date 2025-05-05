cmake_minimum_required(VERSION 3.10)
project(tobas_gazebo_ros)

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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_gazebo_common REQUIRED)
find_package(tobas_gazebo_msgs REQUIRED)

set(AMENT_DEPENDENCIES
    sensor_msgs
    std_srvs
    controller_manager_msgs
    tobas_node
    tobas_drone_msgs_adapter
    tobas_msgs_adapter
    tobas_gazebo_common
    tobas_gazebo_msgs
)

include_directories(include)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "CpuHandlerNode" EXECUTABLE cpu_handler)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "RotorStatesPublisherNode" EXECUTABLE rotor_states_publisher
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "ElectricRotorCommandHandlerNode" EXECUTABLE electric_rotor_command_handler
)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "JointStatesBridge" EXECUTABLE joint_states_bridge)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "JointCommandHandlerNode" EXECUTABLE joint_command_handler)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
