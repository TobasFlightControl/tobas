cmake_minimum_required(VERSION 3.25)
project(tobas_std_msgs_adapter)

set(AMENT_DEPENDENCIES rclcpp tobas_std_tools tobas_std_msgs)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(
  ${PROJECT_NAME} INTERFACE rclcpp::rclcpp tobas_std_tools::tobas_std_tools tobas_std_msgs::tobas_std_msgs
)

install(
  TARGETS ${PROJECT_NAME}
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
