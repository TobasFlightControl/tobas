cmake_minimum_required(VERSION 3.25)
project(tobas_dummy_pkg)

set(AMENT_DEPENDENCIES rclcpp rclcpp_components)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::DummyNode" EXECUTABLE controller)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::DummyNode" EXECUTABLE observer)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::DummyNode" EXECUTABLE mission_executor)

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
