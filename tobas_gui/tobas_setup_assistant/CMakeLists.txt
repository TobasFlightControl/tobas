cmake_minimum_required(VERSION 3.10)
project(tobas_setup_assistant)

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

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tobas_kdl_conversions REQUIRED)
find_package(tobas_linux REQUIRED)
find_package(tobas_property_tools REQUIRED)
find_package(tobas_tools REQUIRED)
find_package(tobas_drone_msgs_adapter REQUIRED)
find_package(tobas_qt_tools REQUIRED)

set(AMENT_DEPENDENCIES
  visualization_msgs
  moveit_msgs
  tobas_kdl_conversions
  tobas_linux
  tobas_property_tools
  tobas_tools
  tobas_drone_msgs_adapter
  tobas_qt_tools
)

# add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

file(GLOB_RECURSE MOC_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/[^.]*.hpp)
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../../third_party/rapidcsv/src
  ${PROJECT_SOURCE_DIR}/../../third_party/inja/include
  ${PROJECT_SOURCE_DIR}/../../third_party/inja/third_party/include
)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} SHARED ${MOC_SOURCES} ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

add_executable(main nodes/main.cpp)
target_link_libraries(main ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY config data launch resources templates DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
