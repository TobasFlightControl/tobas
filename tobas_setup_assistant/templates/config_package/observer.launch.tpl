<launch>

  <arg name="nodelet" default="false"/>

  <group ns="{{ drone_name }}">
    <!-- Load parameters -->
    <rosparam file="$(find {{ config_pkg_name }})/config/observer.yaml" command="load"/>

    <!-- Bringup observer -->
    <include file="$(find {{ observer_pkg }})/launch/observer.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
      <arg name="node_name" value="observer"/>
    </include>
  </group>

</launch>
