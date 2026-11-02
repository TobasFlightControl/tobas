cmake_minimum_required(VERSION 3.25)
project(tobas_qwt_vendor)

find_package(ament_cmake REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent PrintSupport Svg OpenGLWidgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB QWT_SOURCES CONFIGURE_DEPENDS vendor/src/*.cpp vendor/src/*.h)
# QGLWidget was removed in Qt 6; Qwt's replacement uses QOpenGLWidget.
list(FILTER QWT_SOURCES EXCLUDE REGEX "qwt_plot_glcanvas\\.(cpp|h)$")
add_library(qwt SHARED ${QWT_SOURCES})
set_target_properties(qwt PROPERTIES OUTPUT_NAME tobas_qwt_qt6 VERSION 6.3.0 SOVERSION 6)
target_include_directories(
  qwt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/src> $<INSTALL_INTERFACE:include>
)
target_link_libraries(qwt PUBLIC Qt6::Widgets Qt6::Concurrent Qt6::PrintSupport Qt6::Svg Qt6::OpenGLWidgets)

install(TARGETS qwt EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib)
install(
  DIRECTORY vendor/src/
  DESTINATION include/qwt
  FILES_MATCHING
  PATTERN "*.h"
)
install(FILES vendor/COPYING README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Qt6Widgets Qt6Concurrent Qt6PrintSupport Qt6Svg Qt6OpenGLWidgets)
ament_package()
