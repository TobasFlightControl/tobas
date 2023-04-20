# Tobas

## Installation and Build

### Ubuntu 20.04LTS

```bash
$ cd (catkin_workspace)/src
$ git clone https://github.com/Masa0u0/tobas.git
$ rosdep install --from-paths . --ignore-src -ry
$ pip install -r tobas/requirements.txt
$ catkin build
```

## Basic Usage

### Create Tobas configuration package

```bash
$ roslaunch tobas_setup_assistant setup_assistant.launch
```

### Gazebo Simulation

```bash
$ roslaunch (tobas_config_pkg) gazebo.launch
```

### Bringup observer and controller

```bash
$ roslaunch (tabas_config_pkg) bringup.launch
```

For controller API, please see [controllers](https://github.com/Masa0u0/tobas/tree/main/controllers#readme).

### Teleoperation

```bash
$ roslaunch tobas_keyboard_teleop keyboard_teleop.launch  # By keyboard
$ roslaunch tobas_gui_teleop gui_teleop.launch            # By GUI application
```
