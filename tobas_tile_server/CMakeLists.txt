cmake_minimum_required(VERSION 3.10)
project(tobas_tile_server)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(DIRECTORY
  src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE
)
file(GLOB_RECURSE SCRIPTS RELATIVE ${PROJECT_SOURCE_DIR} scripts/[^.]*)
catkin_install_python(PROGRAMS ${SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
