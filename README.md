# Tobas

## Installation - Ubuntu 20.04 LTS with ROS Noetic

1. Install ROS Noetic

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install -y ros-noetic-desktop-full ros-noetic-ros-controllers ros-noetic-gazebo-ros-control python3-rosdep python3-rosinstall python3-catkin-tools python3-vcstool python3-pip
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
$ git clone git@github.com:TobasFlightControl/tobas.git -b main --recursive
$ vcs import . < tobas/.rosinstall_main --recursive
$ rosdep install --from-paths . --ignore-src -ry
$ pip install -r tobas/requirements.txt
```

4. Install [QuadProgpp](https://github.com/liuq/QuadProgpp) and [qpOASES](https://github.com/coin-or/qpOASES)

```bash
# Install QuadProgpp
$ git clone https://github.com/liuq/QuadProgpp.git
$ cd QuadProgpp
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
$ make -j
$ sudo make install
```

```bash
# Install qpOASES
$ git clone https://github.com/coin-or/qpOASES.git -b stable/3.2
$ cd qpOASES
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
$ make -j
$ sudo make install
```

5. Build catkin workspace

```bash
$ catkin build
```

## Basic Usage

### 1. Create Tobas configuration package using setup assistant.

```bash
$ roslaunch tobas_setup_assistant setup_assistant.launch
```

Examples of robot description can be found in `tobas/tobas_description/urdf/`.

### 2. Launch your drone

Launch the drivers for the drone's sensors and propulsing system in Gazebo or in the real world.

#### In the case of Gazebo simulation

```bash
$ roslaunch (tobas_config_pkg) gazebo.launch  # Wait until the drone stops
$ roslaunch (tobas_config_pkg) bringup.launch  # Launch Tobas control software
```

#### In the case of the real world

1. Connect FC and an external PC to the same network

2. Send configuration package from the PC to FC

```bash
$ scp -r ~/catkin_ws/src/(tabas_config_pkg)/ (user)@(host):/home/(user)/catkin_ws/src/
```

3. SSH into FC

```bash
$ ssh (user)@(host)
```

4. Execute real.launch with superuser privileges

Please make sure that the RC transmitter and receiver can communicate correctly.

```bash
$ su
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch (tobas_config_pkg) real.launch
```

### 3. Teleoperation

```bash
$ roslaunch (tobas_config_pkg) keyboard_teleop.launch  # By keyboard
$ roslaunch (tobas_config_pkg) gui_teleop.launch       # By GUI application
```

## Advanced usage

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

### Hardware in the Loop (HIL)

<strong>Warning: This operation will rotate the motors.</strong>

1. Launch roscore on FC.

```bash
$ roscore
```

2. Launch Gazebo simulation on the external PC.

```bash
$ roslaunch (tobas_config_pkg) gazebo.launch
```

3. Launch HIL software on FC.\
   Make sure battery and motors are connected to FC properly and the propellers are NOT attached to the motors.

```bash
$ su
$ roslaunch (tobas_config_pkg) hil.launch
```

## Calibration

### Accelerometer Calibration

```bash
$ ~/catkin_ws/devel/lib/tobas_real/accel_calibration
```

### Magnetometer Calibration

```bash
$ ~/catkin_ws/devel/lib/tobas_real/mag_calibration
```

### ADC Calibration

Make sure battery is connected to FC properly.\
Execute the following in FC:

```bash
$ ~/catkin_ws/devel/lib/tobas_real/adc_calibration
```

### RC Input Calibration

Make sure RC receiver is connected to FC properly and it can communicate with a transmitter.\
Execute the following in FC:

```bash
$ ~/catkin_ws/devel/lib/tobas_real/rcin_calibration
```

### ESC Calibration

Make sure battery and ESCs are connected to FC properly and the propellers are NOT attached to the motors.\
Execute the following in FC:

```bash
$ su
$ ~/catkin_ws/devel/lib/tobas_real/esc_calibration
```

### Measure sensor noise

<strong>Warning: This operation will rotate the motors.</strong>

The rotation of the propellers has a significant impact on the IMU (Inertial Measurement Unit), so measuring in a state where the propellers are rotating will yield data closer to actual flight conditions. In this case, make sure that the battery, ESC (Electronic Speed Controller), motors, and FC (Flight Controller) are properly connected, and that the airframe is securely fixed to prevent movement.\
Execute the following in FC (Be prepared to press Ctrl+C to immediately stop the program in case of danger):

```bash
$ su
$ ~/catkin_ws/devel/lib/tobas_real/measure_sensor_noise
```

## Trouble Shooting

### Robot meshes not visible [WSL]

With WSL there are still some issues with using GPU, and in particular OpenGL, and this creates problems for visualizing meshes in rviz.\
A temporary fix would be to export:

```bash
$ export LIBGL_ALWAYS_SOFTWARE=1
$ export LIBGL_ALWAYS_INDIRECT=0
```

and if you are using an Xserver, leave "Native opengl" option unchecked. This however will force the system to work on CPU, but that's what we have for now. \
cf. [Robot meshes not visible in rviz [Windows11, WSL2]](https://answers.ros.org/question/394135/robot-meshes-not-visible-in-rviz-windows11-wsl2/)

### Unstable takeoff

1. Have the sensors, ADC, and ESC been calibrated?
1. There might be a large discrepancy between the model and the actual drone.
   - The position of the center of gravity.
   - Sensor positions.
   - Motor mounting angles.
   - Is the inertia tensor defined in the NWU coordinate system?
1. Increase the takeoff speed and/or the gain in the vertical direction to rise quickly.
1. Increase the intensity of the attitude control.
   - Increase the attitude weight.
   - Reduce the decay time constant of the attitude error.
