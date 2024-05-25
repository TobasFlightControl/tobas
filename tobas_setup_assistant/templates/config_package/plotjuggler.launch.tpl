<launch>

  <node ns="{{ drone_name }}" pkg="tobas_rospy" type="plotjuggler_launcher_node.py" name="plotjuggler_launcher" output="screen">
    <param name="required_topics" type="yaml" value="['odom']"/>
    <param name="layout" value="$(find {{ config_pkg_name }})/config/plotjuggler_layout.xml"/>
  </node>

</launch>
