cmake_minimum_required(VERSION 3.31)
project(tobas_random_axis_tilt_multi_controller)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    tobas_nlp
    tobas_node
    tobas_pose_pid
    tobas_tools
    tobas_nonplanar_multi_controller
    tobas_drone_msgs_adapter
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
  tobas_nlp::tobas_nlp
  tobas_node::tobas_node
  tobas_pose_pid::tobas_pose_pid
  tobas_tools::tobas_tools
  tobas_nonplanar_multi_controller::tobas_nonplanar_multi_controller
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_debug_msgs_adapter::tobas_debug_msgs_adapter
)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_TARGETS})
tobas_target_defaults(${PROJECT_NAME}_node)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_options(${PROJECT_NAME}_node PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(
  ${PROJECT_NAME}_node PLUGIN "tobas::random_axis_tilt_multicopter::ControllerNode" EXECUTABLE controller
)
tobas_target_defaults(controller)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
