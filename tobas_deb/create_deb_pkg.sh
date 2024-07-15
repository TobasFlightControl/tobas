#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
cd ${TOBAS_DEB}

# Ubuntu
UBUNTU_WS=$(realpath ${TOBAS_DEB}/ubuntu)
catkin build tobas -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${UBUNTU_WS}/opt/tobas/
fakeroot dpkg-deb --build ${UBUNTU_WS} ${TOBAS_DEB}

# Navio2
NAVIO2=$(realpath ${TOBAS_DEB}/navio2)
fakeroot dpkg-deb --build ${NAVIO2} ${TOBAS_DEB}
