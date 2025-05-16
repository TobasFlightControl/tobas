# Tobas T1

## 追加のセットアップ

- ROS2 のインストール: ros2_jazzy_install_raspbian.sh
- raspi-config から I2C と UART を有効化
- gcc のバージョンを 12.2.0 から 13.2.0 にアップグレード: install_gcc.sh (ラズパイ 5，シングルコアで 6 時間ほど)
- PREEMPT_RTカーネルパッチを適用: https://nw-electric.way-nifty.com/blog/2024/10/post-d9d046.html
