# {{ pkg_name }}

## Bringup Gazebo Simulator

```bash
$ roslaunch {{ pkg_name }} gazebo.launch
```

## Bringup Observer and Controller

```bash
$ roslaunch {{ pkg_name }} bringup.launch
```

## Teleoperation

```bash
$ roslaunch tobas_keyboard_teleop position_yaw.launch drone_name:={{ drone_name }}  # Keyboard
$ roslaunch tobas_gui_teleop gui_teleop.launch drone_name:={{ drone_name }}         # GUI
```

## Parameter Tuning

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
$ rosparam dump {{ pkg_name }}/config/controller.yaml /tobas_multirotor_controller
```
