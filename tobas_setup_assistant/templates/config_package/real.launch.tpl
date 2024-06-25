<launch>

  <arg name="nodelet" default="true"/>

  <!-- Load common parameters -->
  <include file="$(find {{ config_pkg_name }})/launch/common_params.launch">
    <arg name="user_debug" value="false"/>
  </include>

  <!-- Launch Tobas core software -->
  <include file="$(find {{ config_pkg_name }})/launch/bringup.launch">
    <arg name="nodelet" value="$(arg nodelet)"/>
  </include>

  <group ns="{{ drone_name }}">
    <!-- FIXME: Tobas should not depend on each hardware. -->
    <include file="$(find tobas_navio_ros)/launch/hardware_interfaces.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch nodes that work only on Raspberry Pi. -->
    <include file="$(find tobas_real_ros)/launch/real.launch">
      <arg name="nodelet" value="$(arg nodelet)"/>
    </include>

    <!-- Launch calibration nodes -->
    <include file="$(find tobas_calibration_ros)/launch/calibrations.launch"/>

    <!-- Launch user launch file -->
    <include file="$(find {{ user_pkg_name }})/launch/real.launch"/>
  </group>

</launch>
