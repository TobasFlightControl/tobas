cmake_minimum_required(VERSION 3.31)
project(tobas_eskf)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    tf2_ros
    tobas_eigen_tools
    tobas_geographic
    tobas_kdl_conversions
    tobas_node
    tobas_msgs_adapter
    tobas_debug_msgs_adapter
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME}
  tf2_ros::tf2_ros
  tobas_eigen_tools::tobas_eigen_tools
  tobas_geographic::tobas_geographic
  tobas_kdl_conversions::tobas_kdl_conversions
  tobas_node::tobas_node
  tobas_msgs_adapter::tobas_msgs_adapter
  tobas_debug_msgs_adapter::tobas_debug_msgs_adapter
)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::ErrorStateKalmanFilterNode" EXECUTABLE eskf_node)
tobas_target_defaults(eskf_node)

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
