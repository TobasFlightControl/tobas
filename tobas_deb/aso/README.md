# Tobas Aso

## 追加のセットアップ

- ROS2 のインストール: ros2_jazzy_install_raspbian.sh
- raspi-config から I2C と UART を有効化
- `/etc/dhcp/dhclient.conf`の`timeout`の行をコメントアウトして 60sec から 5sec に変更
- gcc のバージョンを 12.2.0 から 13.2.0 にアップグレード: install_gcc.sh (ラズパイ 5，シングルコアで 6 時間ほど)
