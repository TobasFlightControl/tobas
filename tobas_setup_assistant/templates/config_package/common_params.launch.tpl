<launch>

  <arg name="user_debug" default="false"/>

  <group ns="{{ drone_name }}">
    <!-- Load drone configurations from TBSF file to parameter server -->
    <rosparam file="$(find {{ config_pkg_name }})/config/drone.tbsdrn" command="load"/>

    <!-- Load the URDF into the ROS parameter server -->
    <param name="robot_description" command="$(find xacro)/xacro '$(find {{ config_pkg_name }})/urdf/drone.xacro' DEBUG:=$(arg user_debug)"/>
  </group>

</launch>
