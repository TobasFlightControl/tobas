cmake_minimum_required(VERSION 3.25)
project(tobas_keyboard_teleop)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES tobas_keyboard tobas_node tobas_msgs_adapter tobas_command_msgs_adapter tobas_mission_items
                       tobas_mission_msgs
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
set(TARGETS_TO_INSTALL "")
foreach(file_path IN LISTS NODE_TARGETS)
  get_filename_component(NAME ${file_path} NAME_WE)
  add_executable(${NAME} ${file_path})
  ament_target_dependencies(${NAME} ${AMENT_DEPENDENCIES})
  list(APPEND TARGETS_TO_INSTALL ${NAME})
endforeach()

install(TARGETS ${TARGETS_TO_INSTALL} DESTINATION lib/${PROJECT_NAME})

ament_package()
