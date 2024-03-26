#!/bin/bash

# ROSの依存解決はdebではできないため，別でインストール用アプリケーションを作成

# ===== Install ROS Noetic: https://wiki.ros.org/ja/noetic/Installation/Ubuntu =====
# 鍵の設定
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 鍵の追加により利用可能になったROS関連のAPTパッケージを反映
sudo apt update
sudo apt upgrade -y

# ROSに依存するパッケージをインストール
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
sudo apt install -y python3-vcstool python3-catkin-tools

# rosdepの初期化
sudo rosdep init
rosdep update

# ===== Install MAVROS =====
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh

# ===== Install balenaEtcher =====
curl -1sLf 'https://dl.cloudsmith.io/public/balena/etcher/setup.deb.sh' | sudo -E bash
sudo apt update
sudo apt install balena-etcher-electron

# ===== Install ROS dependencies of Tobas =====
# pipを使うと依存管理が面倒なので，Pythonパッケージも極力package.xmlに書いてaptで管理する
source /opt/ros/noetic/setup.bash
rosdep install --from-paths /opt/tobas/ --ignore-src -ry

echo "Finished"
