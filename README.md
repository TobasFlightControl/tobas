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
$ vcs import . < tobas/.rosinstall_main --recursive
$ rosdep install --from-paths . --ignore-src -ry
$ pip install -r tobas/requirements.txt
```

4. Install QuadProgpp

```bash
$ cd ~
$ git clone https://github.com/liuq/QuadProgpp.git
$ cd QuadProgpp
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
$ sudo make install
```

5. Build catkin workspace

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

### Launch your drone

#### Gazebo simulation

```bash
$ roslaunch (tobas_config_pkg) gazebo.launch
```

#### Real world

1. Send configuration package from your PC to FC

```bash
$ scp -r ~/catkin_ws/src/(tabas_config_pkg)/ (user)@(host):/home/(user)/catkin_ws/src/
```

2. SSH into FC

```bash
$ ssh (user)@(host)
```

3. Execute real.launch with superuser privileges

```bash
$ su
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch (tobas_config_pkg) real.launch
```

### Bringup observer and controller

```bash
$ roslaunch (tabas_config_pkg) bringup.launch
```

### ESC Calibration

Make sure battery and ESCs are connected to FC properly.\
Execute the following in FC:

```bash
$ su
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch tobas_real esc_calibration.launch
```

### Teleoperation

```bash
$ roslaunch tobas_keyboard_teleop keyboard_teleop.launch drone_name:=(drone_name)  # By keyboard
$ roslaunch tobas_gui_teleop gui_teleop.launch drone_name:=(drone_name)            # By GUI application
```

### Run FC and external PC on the same ROS network

1. Make sure that the FC and the external PC are connected to the same network.
2. Make sure that the ROS versions on the FC and the external PC are the same.
3. On the FC, launch roscore.

```bash
$ roscore
```

4. On the external PC, set the following environment variables:

```bash
$ export ROS_IP=`hostname -I | cut -d' ' -f1`
$ export ROS_HOSTNAME=`hostname -I | cut -d' ' -f1`
$ export ROS_MASTER_URI=http://(IP address of FC):11311
```

5. Confirm that the external PC can communicate with the ROS nodes inside the FC.

```bash
$ rosnode ping /rosout
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
