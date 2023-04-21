# Tobas

## Installation - Ubuntu 20.04 LTS with ROS Noetic

1. Install ROS Noetic

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install -y ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-catkin-tools python3-vcstool ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/noetic/setup.bash
```

2. Create catkin workspace

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

3. Clone Tobas and install dependencies

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Masa0u0/tobas.git
$ vcs import . < tobas/.rosinstall --recursive
$ rosdep install --from-paths . --ignore-src -ry
$ pip install -r tobas/requirements.txt
```

4. Build catkin workspace

```bash
$ catkin build
```

## Basic Usage

### Create Tobas configuration package

```bash
$ cd ~/catkin_ws
$ catkin source
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

## Trouble Shooting

### Robot meshes not visible [WSL]

With WSL there are still some issues with using GPU, and in particular OpenGL, and this creates problems for visualizing meshes in rviz. A temporary fix would be to export:

```bash
$ export LIBGL_ALWAYS_SOFTWARE=1
$ export LIBGL_ALWAYS_INDIRECT=0
```

and if you are using an Xserver, leave "Native opengl" option unchecked. This however will force the system to work on CPU, but that's what we have for now. \
cf. [Robot meshes not visible in rviz [Windows11, WSL2]](https://answers.ros.org/question/394135/robot-meshes-not-visible-in-rviz-windows11-wsl2/)
