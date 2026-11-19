cmake_minimum_required(VERSION 3.31)
project(tobas_fake_publishers)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_node tobas_constants tobas_msgs_adapter)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(
  ${PROJECT_NAME}_node tobas_node::tobas_node tobas_constants::tobas_constants tobas_msgs_adapter::tobas_msgs_adapter
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::FakeBattPublisherNode" EXECUTABLE fake_batt_publisher
)
tobas_target_defaults(fake_batt_publisher)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::FakeGnssPublisherNode" EXECUTABLE fake_gnss_publisher
)
tobas_target_defaults(fake_gnss_publisher)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::FakeRcInputPublisherNode" EXECUTABLE fake_rcin_publisher
)
tobas_target_defaults(fake_rcin_publisher)

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
