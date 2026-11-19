cmake_minimum_required(VERSION 3.31)
project(tobas_fc2xx_test)

find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

set(AMENT_DEPENDENCIES tobas_ic_drivers tobas_mcu_driver tobas_sbus_driver)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

file(GLOB_RECURSE TEST_TARGETS CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} test/*.cpp)
set(TARGETS_TO_INSTALL "")
foreach(file_path IN LISTS TEST_TARGETS)
  get_filename_component(NAME ${file_path} NAME_WE)
  add_executable(${NAME} ${file_path})
  tobas_target_defaults(${NAME})
  target_link_libraries(
    ${NAME} tobas_ic_drivers::tobas_ic_drivers tobas_mcu_driver::tobas_mcu_driver tobas_sbus_driver::tobas_sbus_driver
  )
  list(APPEND TARGETS_TO_INSTALL ${NAME})
endforeach()

install(TARGETS ${TARGETS_TO_INSTALL} DESTINATION lib/${PROJECT_NAME})

ament_package()
