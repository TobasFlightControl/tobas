cmake_minimum_required(VERSION 3.31)
project(tobas_debug_tools)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_dsp tobas_node tobas_msgs_adapter)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(
  ${PROJECT_NAME}_node tobas_dsp::tobas_dsp tobas_node::tobas_node tobas_msgs_adapter::tobas_msgs_adapter
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::ImuNoiseFilter" EXECUTABLE imu_noise_filter)
tobas_target_defaults(imu_noise_filter)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::MatrixEulerConverterNode" EXECUTABLE matrix_euler_converter
)
tobas_target_defaults(matrix_euler_converter)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_package()
