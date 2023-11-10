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

2. Install MAVROS

```bash
$ sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ chmod u+x install_geographiclib_datasets.sh
$ ./install_geographiclib_datasets.sh
```

3. Create catkin workspace

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

4. Clone Tobas and install dependencies

```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:TobasFlightControl/tobas.git -b main --recursive
$ vcs import . < tobas/.rosinstall_main --recursive
$ rosdep install --from-paths . --ignore-src -ry
$ pip install -r tobas/requirements.txt
```

5. Build catkin workspace

```bash
$ catkin build
```

### Other Installation

You can skip each step if you do not use the feature.

#### ArduPilot

```bash
$ git clone https://github.com/ArduPilot/ardupilot.git  # Anywhere
$ cd ardupilot
$ git checkout ArduCopter-stable
$ Tools/environment_install/install-prereqs-ubuntu.sh -y
$ . ~/.profile
$ git submodule update --init --recursive
$ ./waf configure --board sitl
$ ./waf copter
```

## Basic Usage

### 1. Create Tobas configuration package using setup assistant.

```bash
$ roslaunch tobas_setup_assistant setup_assistant.launch
$ cd ~/catkin_ws
$ catkin build your_config_pkg  # Replace "your_config_pkg" with your configuration packege name
```

Examples of robot description can be found in `tobas/tobas_description/urdf/`.

### 2. Launch your drone

Launch the drivers for the drone's sensors and propulsing system in Gazebo or in the real world.

#### In the case of Gazebo simulation

```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch your_config_pkg gazebo.launch   # Launch Gazebo simulation with your drone
$ roslaunch your_config_pkg bringup.launch  # Launch Tobas control software
```

#### In the case of the real world

1. Connect FC and an external PC to the same network

2. Send configuration package from the PC to FC

```bash
# e.g.) scp -r ~/catkin_ws/src/tobas_iris_config/ pi@192.168.1.1:/home/pi/catkin_ws/src/
$ scp -r ~/catkin_ws/src/your_config_pkg/ (user)@(host):/home/(user)/catkin_ws/src/
```

3. SSH into FC

```bash
$ ssh (user)@(host)  # e.g.) ssh pi@192.168.1.1
```

4. Execute real.launch with superuser privileges

Please make sure that the RC transmitter and receiver can communicate correctly.

```bash
$ su
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch your_config_pkg real.launch
```

### 3. Teleoperation

```bash
$ roslaunch your_config_pkg keyboard_teleop.launch  # By keyboard
$ roslaunch your_config_pkg gui_teleop.launch       # By GUI application
```

### 4. Plot data

You can use PlotJuggler to monitor various data in real time. Please execute the following:

```bash
$ roslaunch your_config_pkg plotjuggler.launch
```

In the first dialog, select 'ROS Topic Subscriber', and in the second dialog, select all topics using Ctrl + A.\
You can easily add or remove data to be displayed.
For more details, please visit [PlotJuggler Tutorials](https://slides.com/davidefaconti/introduction-to-plotjuggler).

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

The IP address of FC can be found by executing following within the FC.

```bash
$ hostname -I
>> 192.168.1.1
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
$ roslaunch your_config_pkg gazebo.launch
```

3. Launch HIL software on FC.\
   Make sure battery and motors are connected to FC properly and the propellers are NOT attached to the motors.

```bash
$ su
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch your_config_pkg hil.launch
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
