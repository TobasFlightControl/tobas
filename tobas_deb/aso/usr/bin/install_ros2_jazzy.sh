#!/bin/bash

# cf. https://zenn.dev/thorie/articles/548emb-install-ros2-offical-ver-on-raspberry-pi-os

# Set locale
sudo apt update &&
sudo apt install locales &&
sudo locale-gen en_US en_US.UTF-8 &&
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&
export LANG=en_US.UTF-8 &&

# Enable required repositories
sudo apt install software-properties-common &&
# sudo add-apt-repository universe &&
sudo apt update &&
sudo apt install -y curl &&
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null &&

# Install development tools (optional)
sudo apt update &&
sudo apt upgrade -y &&
sudo apt install -y ros-dev-tools &&
sudo apt install -y python3-colcon-common-extensions &&
# sudo apt install -y python3-flake8-blind-except &&
# sudo apt install -y python3-flake8-class-newline &&
# sudo apt install -y python3-flake8-deprecated &&
# sudo apt install -y python3-mypy &&
# sudo apt install -y python3-pip &&
# sudo apt install -y python3-pytest &&
# sudo apt install -y python3-pytest-cov &&
# sudo apt install -y python3-pytest-mock &&
# sudo apt install -y python3-pytest-repeat &&
# sudo apt install -y python3-pytest-rerunfailures &&
# sudo apt install -y python3-pytest-runner &&
# sudo apt install -y python3-pytest-timeout &&

# Get ROS2 source code
mkdir -p ~/colcon_ws_install/src &&
cd ~/colcon_ws_install &&
vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos ./src &&

# Install dependencies
sudo apt upgrade -y &&
sudo rosdep init | true &&
rosdep update &&
rosdep install -iy --from-paths ./src --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" &&

# Install additional RMW implementations (optional)
# TODO: https://docs.ros.org/en/jazzy/How-To-Guides/Working-with-multiple-RMW-implementations.html

# Build and install
cd ~/colcon_ws_install/ &&
colcon build --install --install-space /opt/ros/jazzy -DCMAKE_BUILD_TYPE=Release &&

# Verify installation is successful
source /opt/ros/jazzy/setup.bash &&
echo -e "\033[32mROS2 Jazzy is installed successfully.\033[0m"
