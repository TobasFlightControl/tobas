<launch>

  <arg name="nodelet" default="true"/>
  <arg name="ground_truth" default="false"/>

  <group ns="{{ drone_name }}">
    <!-- Takeoff action server -->
    <include file="$(find {{ takeoff_pkg }})/launch/takeoff_action_server.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>

    <!-- Land action server -->
    <include file="$(find {{ landing_pkg }})/launch/landing_action_server.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>

    <!-- Move action server -->
    <include file="$(find {{ move_pkg }})/launch/move_action_server.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>
  </group>

</launch>
