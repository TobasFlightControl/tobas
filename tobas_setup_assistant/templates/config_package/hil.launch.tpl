<launch>

  <group ns="{{ drone_name }}">
    <include file="$(find tobas_navio_ros)/launch/rcin_handler.launch">
      <arg name="nodelet" value="false"/>
    </include>

    <include file="$(find tobas_real_ros)/launch/real.launch">
      <arg name="nodelet" value="false"/>
    </include>
  </group>

</launch>
