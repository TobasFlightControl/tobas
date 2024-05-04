#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
cd ${TOBAS_DEB}

# Ubuntu
UBUNTU_WS=$(realpath ${TOBAS_DEB}/ubuntu)
catkin build tobas -DCMAKE_INSTALL_PREFIX=${UBUNTU_WS}/opt/tobas/
fakeroot dpkg-deb --build ${UBUNTU_WS} ${TOBAS_DEB}

# Raspbian
RASPBIAN_WS=$(realpath ${TOBAS_DEB}/raspbian)
fakeroot dpkg-deb --build ${RASPBIAN_WS} ${TOBAS_DEB}
