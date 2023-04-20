# {{ pkg_name }}

## Bringup Gazebo Simulator

---

```bash
$ roslaunch {{ pkg_name }} gazebo.launch
```

## Bringup Observer and Controller

---

```bash
$ roslaunch {{ pkg_name }} bringup.launch
```

## Teleoperation

---

```bash
$ roslaunch tobas_keyboard_teleop keyboard_teleop.launch  # Keyboard
$ roslaunch tobas_gui_teleop gui_teleop.launch            # GUI
```

## Parameter Tuning

---

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
$ rosparam dump {{ pkg_name }}/config/controller.yaml /tobas_controller
```
