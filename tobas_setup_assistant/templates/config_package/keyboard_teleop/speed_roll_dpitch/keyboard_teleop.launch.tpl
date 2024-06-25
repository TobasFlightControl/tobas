<launch>

  <arg name="ground_truth" default="false"/>

  <group ns="{{ drone_name }}">
    <node pkg="tobas_keyboard_teleop" type="speed_roll_dpitch_publisher_node" name="keyboard_teleop" output="screen">
      <remap if="$(arg ground_truth)" from="odom" to="ground_truth/odom"/>
    </node>
  </group>

</launch>
