cmake_minimum_required(VERSION 3.0.2)
project(tobas_rviz_plugin)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_visualization
  class_loader
  pluginlib
  roscpp
  rviz
)
find_package(Boost REQUIRED thread date_time system filesystem)
find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories(${OGRE_LIBRARY_DIRS})

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS robot_state_rviz_plugin/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_visualization class_loader pluginlib roscpp rviz
)

include_directories(
  robot_state_rviz_plugin/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${QT_INCLUDE_DIR} ${OGRE_INCLUDE_DIRS})

add_subdirectory(robot_state_rviz_plugin)

install(FILES robot_state_rviz_plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
