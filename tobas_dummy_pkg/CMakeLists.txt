cmake_minimum_required(VERSION 3.25)
project(tobas_dummy_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

set(AMENT_DEPENDENCIES rclcpp rclcpp_components)

file(GLOB_RECURSE NODE_CPP_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DummyNode" EXECUTABLE controller)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DummyNode" EXECUTABLE observer)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DummyNode" EXECUTABLE takeoff_server)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DummyNode" EXECUTABLE land_server)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "DummyNode" EXECUTABLE move_server)

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
