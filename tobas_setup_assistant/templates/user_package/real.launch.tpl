<!-- Do not delete or rename this file because it is executed in {{ config_pkg_name }}/real.launch. -->

<launch>

  <arg name="nodelet" default="false"/>

  <!-- Bringup Tobas bridge -->
  <node if="$(arg nodelet)" pkg="nodelet" type="nodelet" name="tobas_bridge" args="load TobasBridgeNodelet nodelet_manager" output="screen" required="true"/>
  <node unless="$(arg nodelet)" pkg="{{ user_pkg_name }}" type="tobas_bridge_node" name="tobas_bridge" output="screen" required="true"/>

  <!-- Please launch the nodes that run only on the actual machine. -->

</launch>
