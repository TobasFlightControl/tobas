cmake_minimum_required(VERSION 3.31)
project(tobas_config_servers)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_node tobas_constants tobas_msgs_adapter)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME} SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME} tobas_node::tobas_node tobas_constants::tobas_constants tobas_msgs_adapter::tobas_msgs_adapter
)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "tobas::ImuFilterConfigServer" EXECUTABLE imu_filter_config_server
)
tobas_target_defaults(imu_filter_config_server)
rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "tobas::RpmControlConfigServer" EXECUTABLE rpm_control_config_server
)
tobas_target_defaults(rpm_control_config_server)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_package()
