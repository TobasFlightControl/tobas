cmake_minimum_required(VERSION 3.25)
project(tobas_gamepad_ros)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBEVDEV REQUIRED libevdev)

set(AMENT_DEPENDENCIES
    rclcpp
    rclcpp_components
    tobas_constants
    tobas_msgs
    tobas_msgs_adapter
    tobas_node
    tobas_gamepad
)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}_node SHARED nodes/rc_input_publisher.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE ${LIBEVDEV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})

target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)

rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::tobas_gamepad::RcInputPublisher" EXECUTABLE rc_input_publisher
)

install(
  TARGETS ${PROJECT_NAME}_node rc_input_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
