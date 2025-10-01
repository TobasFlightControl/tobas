#!/bin/bash

set -e

# Set paths
TOBAS_DEB=$(realpath $(dirname "$0"))
UBUNTU_WORKSPACE=${TOBAS_DEB}/ubuntu
TMP_BUILD_DIR=${ROS2_WORKSPACE}/tmp # Temporal build directory
INSTALL_BASE=${UBUNTU_WORKSPACE}/opt/tobas
BUILD_BASE=${TMP_BUILD_DIR}/build

# Navigate to the colcon workspace
cd ${ROS2_WORKSPACE}

# Set log path
export COLCON_LOG_PATH=${TMP_BUILD_DIR}/log

# Build in the temporal build directory
colcon build --merge-install --packages-up-to tobas --parallel-workers $(nproc) --install-base ${INSTALL_BASE} --build-base ${BUILD_BASE} --cmake-args -DCMAKE_BUILD_TYPE=Release

# Create deb package
fakeroot dpkg-deb --build ${UBUNTU_WORKSPACE} ${TOBAS_DEB}

# Remove generated objects
rm -rf ${TMP_BUILD_DIR} ${UBUNTU_WORKSPACE}/opt/tobas
