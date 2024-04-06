# Tobas

## ラズパイの設定

### CPU クロックの設定 (CPU 冷却必須)

cf. [RaspberryPi4 の高速化（オーバークロック）](https://qiita.com/ousagi_sama/items/67ea6c7332df8d23b842)
cf. [「Raspberry Pi」をオーバークロックしてみた](https://japan.zdnet.com/article/35201090/)

`/boot/config.txt`に以下を追記:

```txt
over_voltage=0    # CPU,GPUへの印加電圧, default: 0, minimum: -16, maximum: 8
arm_freq=1200     # 最大周波数MHz, default: 1200, maximum: 2147
arm_freq_min=1200 # 最小周波数MHz, default: 600, maximum: 2147
gpu_freq=500      # GPU周波数MHz, default: 500, maximum: 750
force_turbo=0     # arm_freq=1200, 1にするとover_voltage=6が強制される
```

#### 注意

CPU や GPU をオーバークロックする場合は over_voltage を正にすることが推奨されるが，
そうするとラズパイと電源を共有する IC が低電圧状態になってしまう (特に Ublox の GNSS レシーバ)．
これは Type-C から給電しようが BEC を使おうが変わらなかった．
そのため，基本的には標準の電圧とクロック数で使用すべきだろう．

### root 権限なしで PWM にアクセスできるようにする

1. ルールの追加

`/etc/udev/rules.d/10-local.rules`に以下を追記 \
cf. [Need to configure non-root PWM access](https://community.emlid.com/t/need-to-configure-non-root-pwm-access/16501/10)

```txt
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
        chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm;\
        chown -R root:gpio /sys/devices/platform/soc/*.spi/spi_master/spi1/spi1.0/pwm/pwmchip0 && chmod -R 770 /sys/devices/platform/soc/*.spi/spi_master/spi1/spi1.0/pwm/pwmchip0\
'"
```

2. udev ルールの適用

```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

### root 権限なしで Dynamixel の USB 通信レイテンシを変更できるようにする

1. ルールの追加

`/etc/udev/rules.d/10-local.rules`に以下を追記

```txt
KERNEL=="ttyUSB0", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:dialout /sys/bus/usb-serial/devices/%k/latency_timer; chmod 770 /sys/bus/usb-serial/devices/%k/latency_timer'"
```

`ttyUSB0`デバイスがシステムに追加された（`ACTION=="add"`）時に，
`latency_timer`ファイルの所有者を`root`ユーザーと`dialout`グループに変更し，
所有者とグループのみに完全なアクセス権を与えている（`chmod 770`）．

2. udev ルールの適用

```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

3. ユーザを dialout グループに追加

```bash
$ sudo usermod -a -G dialout pi
```

4. グループを確認

```bash
$ getent group dialout  # dialoutグループのメンバーを確認
$ id pi                 # piが所属するグループを確認
```

### ラズパイをアクセスポイント&ルーター化

#### 手順

1. [Raspberry Pi WiFi アクセスポイント+クライアント同時使用](https://www.mikan-tech.net/entry/raspi-wifi-ap-sta)
2. [Raspberry Pi WiFi アクセスポイント&ルーター化](https://www.mikan-tech.net/entry/raspi-ap-sta-router)

#### メモ

- `$ sudo iw phy phy0 interface add ap0 type __ap`はアクセスポイントモードでの仮想 WiFi インターフェースを作成するコマンドだが，
  既にアクセスポイントのインターフェースが作成されていたら`command failed: Device or resource busy (-16)`というエラーが出る．
  その場合は hostapd と DHCP を無効化し，固定 IP の設定を削除してからやり直す必要がある．

- `/etc/udev/rules.d/99-ap0.rules`の MAC アドレスをハードコードせず，wlan0 からコピーするよう変更．

```txt
SUBSYSTEM=="ieee80211", ACTION=="add|change", KERNEL=="phy0", \
  RUN+="/sbin/iw phy phy0 interface add ap0 type __ap", \
  RUN+="/bin/ip link set ap0 address $(cat /sys/class/net/wlan0/address)"
```

### Tobas のオート起動のための設定

- `/etc/systemd/system/tobas_xxx.service`にコマンドを書く
  - ExecStart 内でシェルスクリプトを実行して環境変数を設定しても元のシェルには影響しないことに注意
- 環境変数や共通のシェルスクリプトを`/etc/tobas/`以下にまとめる

## CUI で HIL

1. 外部 PC から`raspberry_wifi`に接続

ラズパイの ROS_MASTER_URI を AP のもので固定しているため，それに合わせる必要がある．

2. ラズパイのオート起動のサービスを落とす

```bash
$ sudo systemctl stop tobas_roscore.service
```

3. 外部 PC とラズパイの両方で`fkie_multimaster`を立ち上げる

```bash
$ roslaunch tobas_fkie_master fkie_master.launch
```

4. ラズパイで`rcin_handler`を立ち上げる

```bash
$ roslaunch tobas_navio_ros rcin_handler.launch __ns:=drone_name
```

5. 外部 PC で Tobas ソフトウェアを立ち上げる

```bash
$ roslaunch tobas_iris_config gazebo.launch
$ roslaunch tobas_iris_config bringup.launch
```
