<launch>

  <arg name="num_worker_threads" default="16"/>

  <group ns="{{ drone_name }}">
    <node pkg="nodelet" type="nodelet" name="nodelet_manager" args="manager" output="screen">
      <param name="num_worker_threads" value="$(arg num_worker_threads)"/>
    </node>
  </group>

</launch>
