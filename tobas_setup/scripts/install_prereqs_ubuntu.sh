#!/bin/bash

# Install ROS Noetic: https://wiki.ros.org/ja/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y build-essential python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-vcstool python3-dev python3-pip python3-catkin-tools
sudo rosdep init
rosdep update

# Install MAVROS
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod u+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh

# Install dependencies
rosdep install --from-paths . --ignore-src -ry
pip install numpy sympy pandas pyyaml et-xmlfile jinja2 markdown overrides pyqt-vertical-tab-widget

# Add setup.bash to .bashrc
SETUP_BASH_PATH=$(realpath "./setup.bash")
echo "source ${SETUP_BASH_PATH}" >> ~/.bashrc
