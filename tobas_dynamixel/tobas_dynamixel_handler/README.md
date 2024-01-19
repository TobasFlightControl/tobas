# tobas_dynamixel_handler

## root 権限なしで USB 通信レイテンシを変更できるようにする

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
