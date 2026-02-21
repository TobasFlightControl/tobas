# Tobas FC1xx

## Setup

- OSをSDカードに焼く: [Raspberry Pi OS Lite](https://downloads.raspberrypi.com/raspios_lite_armhf/images/raspios_lite_armhf-2025-12-04/2025-12-04-raspios-trixie-armhf-lite.img.xz)
- SDカードをPCにマウント

```bash
$ sudo mkdir -p /mnt/bootfs /mnt/rootfs
$ sudo mount /dev/sda1 /mnt/bootfs
$ sudo mount /dev/sda2 /mnt/rootfs
```

- ルート以下に各ファイルを配置
- `tobas_bootmedia_config`を用いてネットワークなどの設定
- ディスプレイ，USBキーボード，LANケーブルを接続してラズパイを起動
- `control`に記載の依存パッケージをインストール
- `postinst`に従う
- 一度電源を切り，Wi-Fi経由でSSH接続
- 追加の設定 (下)

## 追加の設定

### raspi-config から I2C と UART を有効化

- I2C: Interface Options -> I2C
- UART: Interface Options -> Serial Port -> Shell: No, Hardware: Yes

### CCACHE

- CCACHEを有効化

```bash
$ sudo apt update && sudo apt install -y ccache
$ mkdir -p ~/.cache/ccache
$ echo "max_size = 5G" > ~/.cache/ccache/ccache.conf  # デフォルトの5GBで十分
$ ccache -s
```

- `~/.bashrc`に以下を追記

```bash
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export CCACHE_DIR="$HOME/.cache/ccache/"
```

### ROS 2 をインストール

`tobas/tobas_dev_tools/scripts/install_ros2_raspbian`
