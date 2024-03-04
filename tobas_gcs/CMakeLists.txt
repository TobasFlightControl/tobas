cmake_minimum_required(VERSION 3.10)
project(tobas_gcs)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
  tobas_tools
)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tobas_tools
)

include_directories(${catkin_INCLUDE_DIRS} include)

set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Network)
add_definitions(-DQT_NO_KEYWORDS) # signals, slots, emitなどのキーワードを無効化し，Q_SIGNALS, Q_SLOTS, Q_EMITの使用を強制
add_definitions(-DLINUX -DPOSIX) # LinuxまたはPOSIX準拠のシステムでコンパイルされることを示す

# MOC (Meta-Object Compiler) でQtオブジェクトの通信やプロパティ管理に関するライブラリを生成
# FIXME: set(CMAKE_AUTOMOC ON)が機能しない
file(GLOB_RECURSE LIB_HPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} include/${PROJECT_NAME}/[^.]*.hpp)
qt5_wrap_cpp(MOC_FILES ${LIB_HPP_FILES})

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${MOC_FILES} ${UIC_FILES} ${LIB_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
set(targets_to_install "")
foreach(FILE IN LISTS EXE_FILES)
  get_filename_component(NODE_NAME ${FILE} NAME_WE)
  add_executable(${NODE_NAME} ${FILE})
  target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
  list(APPEND targets_to_install ${NODE_NAME})
endforeach()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${targets_to_install}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
