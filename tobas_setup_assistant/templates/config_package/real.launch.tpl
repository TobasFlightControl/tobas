<launch>

  <arg name="nodelet" default="true"/>

  <!-- Load common parameters -->
  <include file="$(find {{ config_pkg_name }})/launch/common_params.launch">
    <arg name="user_debug" value="false"/>
  </include>

  <!-- FIXME: Tobas should not depend on each hardware. -->
  <group ns="{{ drone_name }}">
    <include file="$(find tobas_navio_ros)/launch/hardware_interfaces.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>
  </group>

  <!-- Launch nodes that work only on Raspberry Pi. -->
  <group ns="{{ drone_name }}">
    <include file="$(find tobas_real_ros)/launch/real.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>
  </group>

  <!-- Launch Tobas core software -->
  <include file="$(find {{ config_pkg_name }})/launch/bringup.launch">
    <arg name="nodelet" value="$(arg nodelet)"/>
  </include>

  <!-- Launch calibration nodes -->
  <include file="$(find {{ config_pkg_name }})/launch/calibration.launch"/>

  <!-- Launch user launch file -->
  <group ns="{{ drone_name }}">
    <include file="$(find {{ user_pkg_name }})/launch/real.launch"/>
  </group>

</launch>
