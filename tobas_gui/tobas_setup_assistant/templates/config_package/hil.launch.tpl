<launch>

  <!-- Launch Tobas core software -->
  <!-- TODO: Improve communication latency and launch bringup.launch on the flight controller side. -->
  <!-- <include file="$(find {{ config_pkg_name }})/launch/bringup.launch">
    <arg name="nodelet" value="$(arg nodelet)"/>
  </include> -->

  <group ns="{{ drone_name }}">
    <!-- Launch minimal hardware interfaces -->
    <include file="$(find {{ hardware_pkg }})/launch/rcin_handler.launch">
      <arg name="nodelet" value="false"/>
    </include>
  </group>

</launch>
