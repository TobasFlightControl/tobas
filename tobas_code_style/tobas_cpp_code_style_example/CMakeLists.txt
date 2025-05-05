cmake_minimum_required(VERSION 3.10)
project(tobas_cpp_code_style_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tobas_math REQUIRED)
find_package(tobas_msgs_adapter REQUIRED)

set(AMENT_DEPENDENCIES rclcpp rclcpp_components std_msgs mavros_msgs tobas_math tobas_msgs_adapter)

include_directories(include)

file(GLOB_RECURSE LIB_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} src/[^.]*.cpp)
add_library(${PROJECT_NAME} STATIC ${LIB_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

file(GLOB_RECURSE NODE_CPP_FILES RELATIVE ${PROJECT_SOURCE_DIR} nodes/[^.]*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::Talker" EXECUTABLE talker)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "tobas::Listener" EXECUTABLE listener)

file(GLOB_RECURSE EXE_FILES RELATIVE ${PROJECT_SOURCE_DIR} examples/[^.]*.cpp)
set(TARGETS_TO_INSTALL "")
foreach(file_path IN LISTS EXE_FILES)
  get_filename_component(NAME ${file_path} NAME_WE)
  add_executable(${NAME} ${file_path})
  target_link_libraries(${NAME} ${PROJECT_NAME})
  list(APPEND TARGETS_TO_INSTALL ${NAME})
endforeach()

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
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${TARGETS_TO_INSTALL} DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
