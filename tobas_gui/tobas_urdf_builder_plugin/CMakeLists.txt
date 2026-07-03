cmake_minimum_required(VERSION 3.25)
project(tobas_urdf_builder_plugin)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable warnings about Ogre
add_compile_options(-Wno-register)
add_compile_options(-Wno-deprecated-copy)
add_compile_options(-Wno-unused-but-set-parameter)

set(AMENT_DEPENDENCIES
    Qt5Core
    Qt5Widgets
    rviz_common
    rviz_default_plugins
    rviz_rendering
    tobas_std_tools
    tobas_ros2_tools
    tobas_urdf
    tobas_property_client
    tobas_constants
    tobas_qt_tools
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Style that does not use `CMAKE_AUTOMOC` or `CMAKE_AUTOUIC`.

file(GLOB_RECURSE MOC_HEADERS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/ui/*.hpp)
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

file(GLOB_RECURSE UI_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} ui/*.ui)
qt5_wrap_ui(UIC_FILES ${UI_FILES})

include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # colcon_ws/build/${PROJECT_NAME}/

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${MOC_SOURCES} ${UIC_FILES} ${LIB_TARGETS})
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
