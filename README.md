# Tobas

## Setup from source (Ubuntu 24.04 LTS)

1. Install prerequisites

```bash
$ sudo apt update
$ sudo apt install -y curl build-essential locales software-properties-common python3-dev python3-pip
```

2. Set locale

```bash
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
```

3. Install ROS 2 Jazzy

```bash
yes "" | sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-dev-tools
sudo apt install -y ros-jazzy-desktop-full
sudo rosdep init
rosdep update
```

4. Create colcon workspace

```bash
$ mkdir -p ~/colcon_ws/src
$ cd ~/colcon_ws
```

5. Set up your system to source your colcon workspace automatically each time a new shell is opened (optional)

```bash
$ echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
$ echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
$ exec bash
```

6. Clone Tobas

```bash
$ cd ~/colcon_ws/src
$ git clone git@github.com:TobasFlightControl/tobas.git -b main
```

7. Install ROS dependencies

```bash
$ rosdep install --from-paths ~/colcon_ws/src/tobas -yi
```

8. Build

```bash
$ cd ~/colcon_ws
$ colcon build --packages-up-to tobas
```

## CLI Interfaces

### Launch Tobas GCS

```bash
$ ros2 launch tobas_gui_core gui.launch.py
```

### SITL (Simulation in the Loop)

1. Launch Gazebo simulator

```bash
$ ros2 launch ${TOBAS_PACKAGE}_config gazebo.launch
```

2. Launch GUI teleoperation (Optional)

```bash
$ ros2 launch ${TOBAS_PACKAGE}_config gui_teleop.launch
```
