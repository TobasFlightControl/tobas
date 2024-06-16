<launch>

  <group ns="{{ drone_name }}">
    <node pkg="nodelet" type="nodelet" name="nodelet_manager_low" args="manager" output="screen">
      <param name="num_worker_threads" value="8"/>
    </node>
  </group>

</launch>
