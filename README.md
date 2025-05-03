# Tobas

Tobas is a next-generation, model-based flight controller created to meet the demands of the rapidly expanding drone market,
where aircraft are becoming larger and increasingly specialized.
Unlike traditional controllers, Tobas takes each airframe’s unique physical properties into account when designing its control system,
enabling precise flight performance even for unconventional frames not supported by conventional solutions.

## Documentation

[Tobas User Guide](https://tobas-wiki-ja.readthedocs.io/ja/stable/)

## Setup from source (Ubuntu 24.04 LTS)

1. [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

```bash
# Set locale
$ sudo apt update
$ sudo apt install -y locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

# Enable required repositories
$ sudo apt install -y software-properties-common
$ yes "" | sudo add-apt-repository universe
$ sudo apt update
$ sudo apt install -y curl
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update
$ sudo apt upgrade -y

# Install development tools (optional)
$ sudo apt install -y ros-dev-tools
$ sudo apt install -y python3-colcon-common-extensions
$ sudo apt install -y python3-colcon-clean

# Install ROS 2
$ sudo apt install -y ros-jazzy-desktop-full

# Initialize rosdep
$ sudo rosdep init
$ rosdep update
```

2. Create colcon workspace

```bash
$ mkdir -p ~/colcon_ws/src
$ cd ~/colcon_ws
```

3. Set up your system automatically each time a new shell is opened

```bash
$ echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
$ echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
$ echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
$ exec bash
```

4. Clone Tobas

```bash
$ cd ~/colcon_ws/src
$ git clone git@github.com:TobasFlightControl/tobas.git --recursive -b jazzy # or "jazzy-develop" if you contribute to Tobas
```

5. Install dependencies

```bash
$ rosdep install --from-paths ~/colcon_ws/src/tobas -yi
$ sudo apt install -y libgit2-dev
```

6. Build

```bash
$ cd ~/colcon_ws
$ colcon build --packages-up-to tobas
```

## Command Line Interfaces

### Launch Tobas GCS

```bash
$ ros2 launch tobas_gui_core gui.launch.py
```

### Launch Gazebo simulation

```bash
$ ros2 launch ${TOBAS_PACKAGE}_config gazebo.launch
```

## TO-DO List (2025)

- [x] Support for active-tilt multicopters
- [ ] Support for engine-driven models
- [ ] Support for variable-pitch multicopters
- [ ] Support for fixed-wing
- [ ] Support for VTOL
- [ ] CM5-based FMU
- [ ] Visual Inertial Odometry (VIO)
- [ ] Obstacle avoidance
- [ ] Automatic path planning

## For Contributors

See [Contributing to Tobas](./CONTRIBUTING.md).
