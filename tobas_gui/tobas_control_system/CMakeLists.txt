cmake_minimum_required(VERSION 3.25)
project(tobas_control_system)

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
find_package(Qt5QuickWidgets REQUIRED)
find_package(Qt5Positioning REQUIRED)
find_package(magic_enum_ament REQUIRED)
find_package(tobas_path_tools REQUIRED)
find_package(tobas_ros2_tools REQUIRED)
find_package(tobas_constants REQUIRED)
find_package(tobas_drone_core REQUIRED)
find_package(tobas_std_msgs REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)
find_package(tobas_mission_msgs REQUIRED)
find_package(tobas_qt_tools REQUIRED)

set(AMENT_DEPENDENCIES
    Qt5QuickWidgets
    Qt5Positioning
    magic_enum_ament
    tobas_path_tools
    tobas_ros2_tools
    tobas_constants
    tobas_drone_core
    tobas_std_msgs
    tobas_msgs_adapter
    tobas_mission_msgs
    tobas_qt_tools
)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

file(GLOB_RECURSE MOC_HEADERS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/*.hpp)
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

include_directories(include)

file(GLOB_RECURSE LIB_CPP_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${MOC_SOURCES} ${LIB_CPP_FILES})
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
install(DIRECTORY qml DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
