<launch>

  <arg name="wname" default="basic"/>
  <arg name="debug" default="false"/>
  <arg name="paused" default="false"/>
  <arg name="gui" default="true"/>
  <arg name="verbose" default="true"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="user_debug" default="false"/>
  <arg name="x" default="0.0"/>
  <arg name="y" default="0.0"/>
  <arg name="z" default="0.3"/>
  <arg name="yaw" default="0.0" doc="Initial yaw angle in degrees."/>

  <arg name="pi" value="3.14159265359"/>
  <arg name="yaw_rad" value="$(eval arg('yaw') * arg('pi') / 180)"/>

  <!-- Load common parameters -->
  <include file="$(find {{ config_pkg_name }})/launch/common_params.launch">
    <arg name="user_debug" value="$(arg user_debug)"/>
  </include>

  <!-- Launch a Gazebo world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find tobas_gazebo_ros)/worlds/$(arg wname).world"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="verbose" value="$(arg verbose)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>

  <!-- Launch nodelet managers -->
  <include file="$(find {{ config_pkg_name }})/launch/nodelet_manager_fast.launch"/>
  <include file="$(find {{ config_pkg_name }})/launch/nodelet_manager_medium.launch"/>
  <include file="$(find {{ config_pkg_name }})/launch/nodelet_manager_slow.launch"/>

  <group ns="{{ drone_name }}">
    <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_{{ drone_name }}" output="screen" args="-param robot_description -urdf -model {{ drone_name }} -x $(arg x) -y $(arg y) -z $(arg z) -Y $(arg yaw_rad)"/>

    <!-- Load joint controller configurations from YAML file to parameter server -->
    <rosparam file="$(find {{ config_pkg_name }})/config/joint_control.yaml" command="load"/>

    <!-- Load the joint controllers -->
    <node pkg="controller_manager" type="spawner" name="controller_spawner" output="screen" args="{{ joint_controllers }}"/>

    <!-- Launch Gazebo command handlers -->
    <include file="$(find tobas_gazebo_ros)/launch/command_handlers.launch"/>

    <!-- Launch user launch file -->
    <include file="$(find {{ user_pkg_name }})/launch/gazebo.launch"/>
  </group>

</launch>
