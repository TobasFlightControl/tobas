#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
UBUNTU_WS=$(realpath ${TOBAS_DEB}/tobas_deb_ws)

cd ${TOBAS_DEB}
catkin build tobas -DCMAKE_INSTALL_PREFIX=${UBUNTU_WS}/opt/tobas/
fakeroot dpkg-deb --build ${UBUNTU_WS} ${TOBAS_DEB}
