#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
UBUNTU_WS=${TOBAS_DEB}/ubuntu
cd ${ROS2_WORKSPACE}
colcon build --packages-up-to tobas --parallel-workers $(nproc) --install-base ${UBUNTU_WS}/opt/tobas --build-base ${ROS2_WORKSPACE}/build --cmake-args -DCMAKE_BUILD_TYPE=Release
fakeroot dpkg-deb --build ${UBUNTU_WS} ${TOBAS_DEB}
