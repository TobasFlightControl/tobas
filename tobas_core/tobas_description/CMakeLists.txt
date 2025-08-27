cmake_minimum_required(VERSION 3.25)
project(tobas_description)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)
if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
  message(STATUS "This package is skipped on Raspberry Pi.")
  find_package(ament_cmake REQUIRED)
  ament_package()
  return()
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
