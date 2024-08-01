cmake_minimum_required(VERSION 3.10)
project(tobas_tools_py)

find_package(ament_cmake REQUIRED)

catkin_python_setup()
ament_package()

install(DIRECTORY
	src/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	PATTERN "*.pyc" EXCLUDE
	PATTERN "*.egg-info" EXCLUDE
)
