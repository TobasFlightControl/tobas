cmake_minimum_required(VERSION 3.10)
project(tobas_description)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
