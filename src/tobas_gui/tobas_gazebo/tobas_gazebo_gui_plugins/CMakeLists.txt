cmake_minimum_required(VERSION 3.31)
project(tobas_gazebo_gui_plugins)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(tobas_gazebo_common REQUIRED)
find_package(tobas_gazebo_conversions REQUIRED)
find_package(tobas_gazebo_tools REQUIRED)

include_directories(include)

# Build common library.
file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME} tobas_gazebo_common::tobas_gazebo_common tobas_gazebo_conversions::tobas_gazebo_conversions
  tobas_gazebo_tools::tobas_gazebo_tools
)

# Build each plugin.
file(GLOB PLUGIN_DIR_PATHS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} plugins/*)
set(TARGETS_TO_INSTALL "")
foreach(dir_path IN LISTS PLUGIN_DIR_PATHS)
  get_filename_component(PLUGIN_NAME ${dir_path} NAME_WE)
  qt6_wrap_cpp(${PLUGIN_NAME}_MOC ${dir_path}/${PLUGIN_NAME}.hpp) # Variable names must be unique
  qt_add_resources(${PLUGIN_NAME}_RCC ${dir_path}/${PLUGIN_NAME}.qrc) # Variable names must be unique
  add_library(${PLUGIN_NAME} SHARED ${${PLUGIN_NAME}_MOC} ${${PLUGIN_NAME}_RCC} ${dir_path}/${PLUGIN_NAME}.cpp)
  tobas_target_defaults(${PLUGIN_NAME})
  target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME} gz-sim::gz-sim gz-plugin::register)
  target_link_options(${PLUGIN_NAME} PRIVATE -Wl,--no-undefined)
  list(APPEND TARGETS_TO_INSTALL ${PLUGIN_NAME})
endforeach()

# Install plugins.
install(TARGETS ${TARGETS_TO_INSTALL} EXPORT ${PROJECT_NAME} LIBRARY DESTINATION lib/${PROJECT_NAME}/plugins)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")

# Export plugins.
ament_export_libraries(${TARGETS_TO_INSTALL})

ament_package()
