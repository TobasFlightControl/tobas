set(TOBAS_LIB_NAME tobas_rviz_plugin_render_tools)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_BUILD_DIR}/rviz_plugin_render_tools
)

add_library(${TOBAS_LIB_NAME}
  src/rviz_plugin_render_tools/robot_state_visualization.cpp
  src/rviz_plugin_render_tools/planning_link_updater.cpp
)
set_target_properties(${TOBAS_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${TOBAS_LIB_NAME}
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${TOBAS_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
