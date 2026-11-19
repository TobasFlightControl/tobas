cmake_minimum_required(VERSION 3.31)
project(tobas_urdf_builder_plugin)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    Qt6Core
    Qt6Widgets
    rviz_common
    rviz_default_plugins
    rviz_rendering
    tobas_std_tools
    tobas_urdf
    tobas_qt_tools
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Style that does not use `CMAKE_AUTOMOC` or `CMAKE_AUTOUIC`.

file(GLOB_RECURSE MOC_HEADERS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/ui/*.hpp)
qt6_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

file(GLOB_RECURSE UI_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} ui/*.ui)
qt6_wrap_ui(UIC_FILES ${UI_FILES})

include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # colcon_ws/build/${PROJECT_NAME}/

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${MOC_SOURCES} ${UIC_FILES} ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
target_link_libraries(
  ${PROJECT_NAME}
  Qt6::Core
  Qt6::Widgets
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  rviz_rendering::rviz_rendering
  tobas_std_tools::tobas_std_tools
  tobas_urdf::tobas_urdf
  tobas_qt_tools::tobas_qt_tools
)

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
