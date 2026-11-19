cmake_minimum_required(VERSION 3.31)
project(tobas_gcs)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES
    tobas_ros2_tools
    tobas_cyclonedds_config
    tobas_drone_msgs_adapter
    tobas_kdl_msgs_adapter
    tobas_sensor_calibration
    tobas_actuator_test
    tobas_control_system
    tobas_parameter_tuning
    tobas_flight_log_gui
    tobas_fc_console
    tobas_simulation_gui
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(include)

file(GLOB_RECURSE LIB_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.hpp)
add_library(${PROJECT_NAME} STATIC ${LIB_TARGETS})
tobas_target_defaults(${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME}
  tobas_ros2_tools::tobas_ros2_tools
  tobas_cyclonedds_config::tobas_cyclonedds_config
  tobas_drone_msgs_adapter::tobas_drone_msgs_adapter
  tobas_kdl_msgs_adapter::tobas_kdl_msgs_adapter
  tobas_sensor_calibration::tobas_sensor_calibration
  tobas_actuator_test::tobas_actuator_test
  tobas_control_system::tobas_control_system
  tobas_parameter_tuning::tobas_parameter_tuning
  tobas_flight_log_gui::tobas_flight_log_gui
  tobas_fc_console::tobas_fc_console
  tobas_simulation_gui::tobas_simulation_gui
)

# Match the executable name, desktop file name, and application name without spaces so the icon is displayed correctly
# in the desktop application.
add_executable(main nodes/main.cpp)
tobas_target_defaults(main)
target_link_libraries(main ${PROJECT_NAME})
set_target_properties(main PROPERTIES OUTPUT_NAME "TobasGCS")

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch resources DESTINATION share/${PROJECT_NAME})

ament_package()
