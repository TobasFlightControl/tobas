<launch>

  <group ns="{{ drone_name }}">
    <node pkg="nodelet" type="nodelet" name="nodelet_manager_slow" args="manager" output="screen">
      <param name="num_worker_threads" value="4"/>
    </node>
  </group>

</launch>
