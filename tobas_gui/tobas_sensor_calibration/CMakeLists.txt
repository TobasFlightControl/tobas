cmake_minimum_required(VERSION 3.25)
project(tobas_sensor_calibration)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tobas_eigen_conversions REQUIRED)
find_package(tobas_kdl_conversions REQUIRED)
find_package(tobas_property_tree REQUIRED)
find_package(tobas_drone_core REQUIRED)
find_package(tobas_real_common REQUIRED)
find_package(tobas_real_msgs REQUIRED)
find_package(tobas_qt_tools REQUIRED)
find_package(tobas_rqt_bridge REQUIRED)
find_package(tobas_rviz_wrapper REQUIRED)

set(AMENT_DEPENDENCIES
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tobas_eigen_conversions
    tobas_kdl_conversions
    tobas_property_tree
    tobas_drone_core
    tobas_real_common
    tobas_real_msgs
    tobas_qt_tools
    tobas_rqt_bridge
    tobas_rviz_wrapper
)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
