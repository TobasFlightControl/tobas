cmake_minimum_required(VERSION 3.25)
project(tobas_gazebo_sim)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch models worlds DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")
ament_package()
