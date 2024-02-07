<launch>

  <arg name="nodelet" default="false"/>
  <arg name="ground_truth" default="false"/>

  <group ns="{{ drone_name }}">
    <include file="$(find tobas_manipulation)/launch/position_controller.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>
    <include file="$(find tobas_manipulation)/launch/velocity_controller.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>
    <include file="$(find tobas_manipulation)/launch/effort_controller.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="ground_truth" value="$(arg ground_truth)"/>
    </include>
  </group>

</launch>
