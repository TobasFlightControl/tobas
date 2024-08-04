# Tobas

## Setup from source (Ubuntu 20.04 LTS)

1. Install prerequisites

```bash
$ sudo apt instal -y curl, build-essential, policykit-1, python3-dev, python3-pip, python3-et-xmlfile, python3-pyqt5.qtpositioning
```

2. Install ROS Noetic

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ curl -sLf https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt upgrade -y
$ sudo apt install -y ros-noetic-desktop-full
$ sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
$ sudo apt install -y python3-vcstool python3-catkin-tools
$ sudo rosdep init
$ rosdep update
```

3. Create catkin workspace

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

4. Set up your system to source your catkin workspace automatically each time a new shell is opened (optional)

```bash
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ echo "source ~/catkin_ws/src/devel/setup.bash" >> ~/.bashrc
$ exec bash
```

5. Clone Tobas

```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:TobasFlightControl/tobas.git -b develop
```

6. Install ROS dependencies

```bash
$ rosdep install --from-paths ~/catkin_ws/src/tobas -yi
```

7. Build

```bash
$ catkin build -DCMAKE_BUILD_TYPE=Release tobas
```

## CLI Interfaces

### Launch Tobas GCS

```bash
$ roslaunch tobas_gcs gcs.launch
```

### SITL

1. Launch Gazebo simulator

```bash
$ roslaunch ${TOBAS_PACKAGE}_config gazebo.launch
```

2. Launch Tobas flight controller

```bash
$ roslaunch ${TOBAS_PACKAGE}_config bringup.launch
```
