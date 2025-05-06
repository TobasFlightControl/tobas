cmake_minimum_required(VERSION 3.25)
project(tobas_property_client)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tobas_path_tools REQUIRED)
find_package(tobas_ros2_tools REQUIRED)
find_package(tobas_property_common REQUIRED)
find_package(tobas_property_msgs REQUIRED)

set(AMENT_DEPENDENCIES std_srvs tobas_path_tools tobas_ros2_tools tobas_property_common tobas_property_msgs)

include_directories(include)

file(GLOB_RECURSE LIB_CPP_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
