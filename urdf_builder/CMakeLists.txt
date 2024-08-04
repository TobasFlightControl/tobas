cmake_minimum_required(VERSION 3.10)
project(urdf_builder)

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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable warnings about Ogre
add_compile_options(-Wno-register)
add_compile_options(-Wno-deprecated-copy)
add_compile_options(-Wno-unused-but-set-parameter)

find_package(catkin REQUIRED
  tobas_tools
  tobas_linux
  tobas_property_tools
  tobas_std_tools
  rviz
  pluginlib
  urdf
  roslaunch
)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    tobas_tools
    tobas_linux
    tobas_property_tools
    tobas_std_tools
    rviz
    pluginlib
    urdf
    roslaunch
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} # catkin_ws/build/${PROJECT_NAME}/
)

set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-DLINUX -DPOSIX)

file(GLOB_RECURSE MO_FILES RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/ui/[^.]*.hpp)
qt5_wrap_cpp(MOC_FILES ${MO_FILES})

file(GLOB_RECURSE UI_FILES RELATIVE ${PROJECT_SOURCE_DIR} ui/[^.]*.ui)
qt5_wrap_ui(UIC_FILES ${UI_FILES})

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} SHARED ${MOC_FILES} ${UIC_FILES} ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
