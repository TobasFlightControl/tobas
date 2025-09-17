cmake_minimum_required(VERSION 3.25)
project(tobas_manipulation)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(tobas_kdl_conversions REQUIRED)
find_package(tobas_node REQUIRED)
find_package(tobas_tools REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)

set(AMENT_DEPENDENCIES tobas_kdl_conversions tobas_node tobas_tools tobas_msgs_adapter tobas_drone_msgs_adapter)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "PositionControllerNode" EXECUTABLE position_controller)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "VelocityControllerNode" EXECUTABLE velocity_controller)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "EffortControllerNode" EXECUTABLE effort_controller)

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
