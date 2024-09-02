cmake_minimum_required(VERSION 3.10)
project(tobas_dummy_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components)
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
	INCLUDES DESTINATION include
)

ament_package()
