cmake_minimum_required(VERSION 3.10)
project(tobas_gcs)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)
if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
  message("This package is skipped on Raspberry Pi.")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(DIRECTORY config launch resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE
)
file(GLOB_RECURSE SCRIPTS RELATIVE ${PROJECT_SOURCE_DIR} scripts/[^.]*)
catkin_install_python(PROGRAMS ${SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
