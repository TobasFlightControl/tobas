<?xml version="1.0"?>
<package format="3">

	<name>{{ user_pkg_name }}</name>
	<version>0.0.0</version>
	<description>Tobas user package for {{ drone_name }}</description>
	<maintainer email="{{ author_email }}">{{ author_name }}</maintainer>
	<license>BSD</license>

	<buildtool_depend>ament_cmake</buildtool_depend>

	<depend>nodelet</depend>
	<depend>pluginlib</depend>

	<depend>tobas_tools</depend>

	<export>
		<nodelet plugin="${prefix}/nodelet_description.xml"/>
	</export>

	<export>
		<build_type>ament_cmake</build_type>
	</export>

</package>
