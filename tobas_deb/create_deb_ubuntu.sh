#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
WORKSPACE=${TOBAS_DEB}/ubuntu

# ワークスペースに移動
cd ${ROS2_WORKSPACE}

# 過去のビルドディレクトリを削除
# そうしないと自動的にシンボリックリンク生成されることがある
colcon clean workspace -y
rm -rf ${WORKSPACE}/opt/tobas

# ビルド
# シンボリックリンクを作らないように--merge-installオプションをつける
colcon build --merge-install --packages-up-to tobas --parallel-workers $(nproc) --install-base ${WORKSPACE}/opt/tobas --build-base ${ROS2_WORKSPACE}/build --cmake-args -DCMAKE_BUILD_TYPE=Release

# debパッケージを作成
fakeroot dpkg-deb --build ${WORKSPACE} ${TOBAS_DEB}
