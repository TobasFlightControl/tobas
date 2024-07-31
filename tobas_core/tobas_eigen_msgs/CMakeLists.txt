cmake_minimum_required(VERSION 3.10)
project(tobas_eigen_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/[^.]*.msg)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_dependencies(rosidl_default_runtime Eigen3)

ament_package()
