cmake_minimum_required(VERSION 3.31)
project(tobas_ic_drivers_ros)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES rclcpp rclcpp_components geometry_msgs tobas_ic_drivers)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(
  ${PROJECT_NAME}_node rclcpp::rclcpp rclcpp_components::component geometry_msgs::geometry_msgs
  tobas_ic_drivers::tobas_ic_drivers
)
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::Bmm150PublisherNode" EXECUTABLE bmm150_publisher)
tobas_target_defaults(bmm150_publisher)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::Bmm350PublisherNode" EXECUTABLE bmm350_publisher)
tobas_target_defaults(bmm350_publisher)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
