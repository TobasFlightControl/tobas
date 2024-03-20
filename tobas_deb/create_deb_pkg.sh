#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
TOBAS_DEB_WS=$(realpath ${TOBAS_DEB}/tobas_deb_ws)

cd ${TOBAS_DEB}
catkin build tobas -DCMAKE_INSTALL_PREFIX=${TOBAS_DEB_WS}/opt/tobas/
fakeroot dpkg-deb --build ${TOBAS_DEB_WS} ${TOBAS_DEB}
