cmake_minimum_required(VERSION 3.31)
project(tobas_keyboard_teleop)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_keyboard tobas_node tobas_msgs_adapter tobas_command_msgs_adapter tobas_mission_items
                       tobas_mission_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE NODE_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
set(TARGETS_TO_INSTALL "")
foreach(file_path IN LISTS NODE_TARGETS)
  get_filename_component(NAME ${file_path} NAME_WE)
  add_executable(${NAME} ${file_path})
  tobas_target_defaults(${NAME})
  target_link_libraries(
    ${NAME}
    tobas_keyboard::tobas_keyboard
    tobas_node::tobas_node
    tobas_msgs_adapter::tobas_msgs_adapter
    tobas_command_msgs_adapter::tobas_command_msgs_adapter
    tobas_mission_items::tobas_mission_items
    tobas_mission_msgs::tobas_mission_msgs
  )
  list(APPEND TARGETS_TO_INSTALL ${NAME})
endforeach()

install(TARGETS ${TARGETS_TO_INSTALL} DESTINATION lib/${PROJECT_NAME})

ament_package()
