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
pip3 install numpy -U  # Install latest version
pip3 install sympy pandas pyyaml et-xmlfile jinja2 markdown overrides urdf-parser-py pyqt-vertical-tab-widget paramiko scp

# Edit .bashrc
echo "export ROS_IP=\`hostname -I | cut -d' ' -f1\`" >> ~/.bashrc
echo "export ROS_HOSTNAME=\`hostname -I | cut -d' ' -f1\`" >> ~/.bashrc
echo "source $(realpath "./setup.bash")" >> ~/.bashrc
exec bash

# Setup for time synchronization: https://qiita.com/srs/items/ce0a0424e86936fc7170
sudo apt install -y chrony ros-noetic-ntpd-driver  
echo "refclock SHM 0:perm=0666 delay 0.5 refid ROS" | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
