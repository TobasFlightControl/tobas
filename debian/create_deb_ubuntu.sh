#!/bin/bash

set -e

# Set paths.
pardir=$(realpath $(dirname "$0"))
ubuntu_workspace=${pardir}/ubuntu
tmp_build_dir=${ROS2_WORKSPACE}/tmp # Temporal build directory
install_base=${ubuntu_workspace}/opt/ros/jazzy
build_base=${tmp_build_dir}/build

# Navigate to the colcon workspace.
cd ${ROS2_WORKSPACE}

# Set log path.
export COLCON_LOG_PATH=${tmp_build_dir}/log

# Build in the temporal build directory.
colcon build \
  --merge-install \
  --packages-up-to tobas \
  --parallel-workers $(nproc) \
  --install-base ${install_base} \
  --build-base ${build_base} \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Remove colcon-generated prefix-level files and directories that are not package installation destinations.
find "${install_base}" -mindepth 1 -maxdepth 1 \
  ! -name bin \
  ! -name etc \
  ! -name include \
  ! -name lib \
  ! -name share \
  -exec rm -rf -- {} +

# Create deb package.
fakeroot dpkg-deb --build ${ubuntu_workspace} ${pardir}

# Remove generated objects.
rm -rf ${tmp_build_dir} ${ubuntu_workspace}/opt/ros/jazzy
