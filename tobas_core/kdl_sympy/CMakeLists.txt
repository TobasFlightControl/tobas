cmake_minimum_required(VERSION 3.10)
project(kdl_sympy)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()
catkin_package()

install(DIRECTORY
  src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE
)
