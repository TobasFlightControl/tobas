<?xml version="1.0"?>
<package format="2">

	<name>{{ meta_pkg_name }}</name>
	<version>0.0.0</version>
	<description>Tobas meta package for {{ drone_name }}</description>
	<maintainer email="{{ author_email }}">{{ author_name }}</maintainer>
	<license>BSD</license>

	<buildtool_depend>catkin</buildtool_depend>

	<exec_depend>{{ config_pkg_name }}</exec_depend>
	<exec_depend>{{ user_pkg_name }}</exec_depend>

	<export>
		<metapackage/>
	</export>

</package>
