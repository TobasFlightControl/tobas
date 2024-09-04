<!-- Do not delete or rename this file because it is executed in {{ config_pkg_name }}/real.launch. -->

<launch>

  <arg name="nodelet" default="false"/>

  <!-- Bringup Tobas bridge -->
  <node if="$(var nodelet)" pkg="nodelet" exec="nodelet" name="tobas_bridge" args="load TobasBridgeNodelet component_manager_medium" output="screen" required="true"/>
  <node unless="$(var nodelet)" pkg="{{ user_pkg_name }}" exec="tobas_bridge_node" name="tobas_bridge" output="screen" required="true"/>

  <!-- Please launch the nodes that run only on the actual machine. -->

</launch>
