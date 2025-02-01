cmake_minimum_required(VERSION 3.10)
project(tobas_simulation_gui)

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
find_package(tobas_ros2_tools REQUIRED)
find_package(tobas_drone_core REQUIRED)
find_package(tobas_msgs REQUIRED)
find_package(tobas_gazebo_msgs REQUIRED)
find_package(tobas_qt_tools REQUIRED)
find_package(tobas_gui_common REQUIRED)

set(AMENT_DEPENDENCIES
  tobas_ros2_tools
  tobas_drone_core
  tobas_msgs
  tobas_gazebo_msgs
  tobas_qt_tools
  tobas_gui_common
)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

file(GLOB_RECURSE MOC_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/[^.]*.hpp)
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

include_directories(include)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} SHARED ${MOC_SOURCES} ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
