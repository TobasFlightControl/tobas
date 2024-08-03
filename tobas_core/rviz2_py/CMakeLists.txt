cmake_minimum_required(VERSION 3.10)
project(rviz2_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rviz_common REQUIRED)
find_package(python_qt_binding REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

# Include /opt/ros/jazzy/share/python_qt_binding/cmake/sip_helper.cmake
include(${python_qt_binding_DIR}/sip_helper.cmake)

file(GLOB_RECURSE SIP_FILES RELATIVE ${PROJECT_SOURCE_DIR} sip/[^.]*.sip)
file(GLOB_RECURSE RVIZ_COMMON_HEADERS /opt/ros/jazzy/include/rviz_common/rviz_common/[^.]*.hpp)

# Create a dummy target to avoid error
add_custom_target(dummy_target ALL)

build_sip_binding(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/sip/rviz.sip
  SIP_CONFIGURE ${python_qt_binding_DIR}/sip_configure.py
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/sip
  LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib
  DEPENDS ${SIP_FILES} ${RVIZ_COMMON_HEADERS}
  DEPENDENCIES dummy_target
)

install(FILES ${PROJECT_SOURCE_DIR}/lib/librviz_sip.so DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})
