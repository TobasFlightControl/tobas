cmake_minimum_required(VERSION 3.10)
project(tobas_hal_msgs)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tobas_kdl_msgs REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg srv/[^.]*.srv action/[^.]*.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES tobas_kdl_msgs)

add_library(${PROJECT_NAME}_if INTERFACE)

install(
  TARGETS ${PROJECT_NAME}_if
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime tobas_kdl_msgs)

ament_package()
