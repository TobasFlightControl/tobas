#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
DEB_WS=$(realpath ${SCRIPT_DIR}/../tobas_deb_ws)

cd ${DEB_WS}
catkin build tobas -DCMAKE_INSTALL_PREFIX=${DEB_WS}/opt/tobas/
fakeroot dpkg-deb --build ${DEB_WS} ~/Downloads
