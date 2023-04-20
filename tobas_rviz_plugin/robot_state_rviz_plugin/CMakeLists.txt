set(TOBAS_LIB_NAME moveit_robot_state_rviz_plugin)
add_library(${TOBAS_LIB_NAME}_core
  src/robot_state_rviz_plugin/robot_state_display.cpp
  include/robot_state_rviz_plugin/robot_state_display.hpp)
set_target_properties(${TOBAS_LIB_NAME}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${TOBAS_LIB_NAME}_core tobas_rviz_plugin_render_tools
  ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${TOBAS_LIB_NAME} src/robot_state_rviz_plugin/plugin_init.cpp)
set_target_properties(${TOBAS_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${TOBAS_LIB_NAME} ${TOBAS_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${TOBAS_LIB_NAME}_core ${TOBAS_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
