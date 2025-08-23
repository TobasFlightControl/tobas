cmake_minimum_required(VERSION 3.25)
project(tobas_bootmedia_config)

file(READ "/etc/os-release" OS_RELEASE_CONTENT)
if("${OS_RELEASE_CONTENT}" MATCHES "ID=raspbian" OR "${OS_RELEASE_CONTENT}" MATCHES "ID=debian")
  message(STATUS "This package is skipped on Raspberry Pi.")
  find_package(ament_cmake REQUIRED)
  ament_package()
  return()
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(inja_ament REQUIRED)
find_package(tobas_linux REQUIRED)
find_package(tobas_udev REQUIRED)
find_package(wpa_supplicant_parser REQUIRED)
find_package(tobas_qt_tools REQUIRED)

set(AMENT_DEPENDENCIES ament_index_cpp inja_ament tobas_linux tobas_udev wpa_supplicant_parser tobas_qt_tools)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include ${CRYPT_INCLUDE_DIRS})

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${CRYPT_LIBRARIES})

add_executable(main nodes/main.cpp)
target_link_libraries(main ${PROJECT_NAME})
set_target_properties(main PROPERTIES OUTPUT_NAME "TobasBootmediaConfig")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY resources templates DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
