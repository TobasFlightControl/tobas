<launch>

  <arg name="nodelet" default="true"/>
  <arg name="num_worker_threads" default="16"/>
  <arg name="ground_truth" default="false"/>

  <group ns="{{ drone_name }}">
    <!-- Load parameters -->
    <rosparam file="$(find {{ config_pkg_name }})/config/observer.yaml" command="load"/>
    <rosparam file="$(find {{ config_pkg_name }})/config/controller.yaml" command="load"/>
    <rosparam file="$(find {{ config_pkg_name }})/config/dynamic_params.yaml" command="load"/>
    <rosparam file="$(find {{ config_pkg_name }})/config/rc_teleop.yaml" command="load"/>

    <!-- Bringup observer -->
    <include file="$(find {{ observer_pkg }})/launch/observer.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="node_name" value="observer"/>
    </include>

    <!-- Bringup controller -->
    <include file="$(find {{ controller_pkg }})/launch/controller.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
      <arg name="node_name" value="controller"/>
    </include>

    <!-- Launch joint controller -->
    <include file="$(find tobas_manipulation)/launch/controllers.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>

    <!-- Launch state checker -->
    <include file="$(find tobas_state_checker)/launch/state_checker.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch sensor preprocessing modules -->
    <include file="$(find tobas_preprocess)/launch/preprocess.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch pre-arm check server -->
    <include file="$(find tobas_pre_arm_check)/launch/pre_arm_check.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch latency publisher -->
    <include file="$(find tobas_latency_publisher)/launch/latency_publisher.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch robot state publisher -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
      <param name="publish_frequency" value="1000"/>
    </node>

    <!-- Launch RC teleoperation -->
    <include file="$(find tobas_rc_teleop)/launch/rc_teleop.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch action servers for mission -->
    <include file="$(find {{ takeoff_pkg }})/launch/takeoff_action_server.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>
    <include file="$(find {{ landing_pkg }})/launch/landing_action_server.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>
    <include file="$(find {{ move_pkg }})/launch/move_action_server.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>

    <!-- Launch user launch file -->
    <include file="$(find {{ user_pkg_name }})/launch/common.launch"/>
  </group>

</launch>
