<launch>

  <group ns="{{ drone_name }}">
    <include file="$(find tobas_motor_test)/launch/rotor_speeds_publisher.launch"/>
  </group>

  <node pkg="ntpd_driver" type="shm_driver" name="shm_driver" output="screen">
    <param name="shm_unit" value="0"/>
    <param name="fixup_date" value="true"/>
  </node>

</launch>
