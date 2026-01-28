# Tobas FC1xx

## 追加のセットアップ

- raspi-config から I2C と UART を有効化
- gcc のバージョンを 12.2.0 から 13.2.0 にアップグレード: tobas_dev_tools/scripts/install_gcc (ラズパイ 5，シングルコアで 6 時間ほど)
- 更新したコンパイラとライブラリを用いて ROS 2 をインストール: tobas_dev_tools/scripts/install_ros2_raspbian
- PREEMPT_RT カーネルパッチを適用: https://nw-electric.way-nifty.com/blog/2024/10/post-d9d046.html
