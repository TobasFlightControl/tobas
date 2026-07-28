cmake_minimum_required(VERSION 3.25)
project(tobas_gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)

# Derive the vendor package's install prefix from its CMake configuration directory so the Gazebo resource environment
# hook does not depend on a fixed ROS installation path.
get_filename_component(GZ_SIM_VENDOR_PREFIX "${gz_sim_vendor_DIR}/../../.." ABSOLUTE)
message(STATUS "GZ_SIM_VENDOR_PREFIX: ${GZ_SIM_VENDOR_PREFIX}")

install(DIRECTORY launch models worlds DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")
ament_package()
