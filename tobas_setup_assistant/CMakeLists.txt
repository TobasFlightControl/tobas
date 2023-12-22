cmake_minimum_required(VERSION 3.10)
project(tobas_setup_assistant)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()
catkin_package()

install(DIRECTORY config data launch resources templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB_RECURSE SCRIPTS RELATIVE ${PROJECT_SOURCE_DIR} scripts/[^.]*)
catkin_install_python(PROGRAMS ${SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
