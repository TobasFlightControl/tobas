cmake_minimum_required(VERSION 3.31)
project(tobas_cmake NONE)
find_package(ament_cmake REQUIRED)
ament_package(CONFIG_EXTRAS "cmake/tobas_build_settings.cmake")
