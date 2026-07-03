# tobas_gazebo_sim

## Notes

### Documentation

- [Gazebo Tutorials/SDF worlds](https://gazebosim.org/docs/latest/sdf_worlds/)

### Example World Files

- [gz-sim](https://github.com/gazebosim/gz-sim/tree/ign-gazebo3/examples/worlds)
- [Fuel Latest Worlds](https://app.gazebosim.org/fuel/worlds)
  - OpenRobotics models often download meshes separately, so the files themselves can stay lightweight
- [Gazebo Worlds | PX4](https://docs.px4.io/main/en/sim_gazebo_gz/worlds.html)

### URI Specification from Packages

```xml
<include>
  <uri>package://${package_name}/models/${model_name}</uri>
</include>
```

The path must be set correctly in `dsv.in`.

### Download from Fuel, a model-sharing site for Gazebo

```xml
<include>
  <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Sonoma Raceway</uri>
</include>
```
