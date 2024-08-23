<launch>

  <arg name="ground_truth" default="false"/>

  <group ns="{{ drone_name }}">
    <node pkg="tobas_gui_teleop" type="base_pose_commander_node.py" name="base_pose_commander" output="screen">
      <remap if="$(arg ground_truth)" from="odom" to="ground_truth/odom"/>
    </node>
  </group>

</launch>
