# トラブルシューティング

現在までに報告されたトラブルとその解決方法をまとめています．

## Gazebo の動作が重い

---

### 1. ディスプレイサーバが X11 になっていない可能性

`Settings / System / About / System Details`
から，ディスプレイサーバが X11 になっていることを確認してください．

![troubleshooting/system_details](../../assets/troubleshooting/system_details.png)

もしも X11 でない場合，例えば Wayland と表示されている場合は，
Ubuntu の起動画面でユーザ名を選択した際に画面右下に現れる歯車から，ディスプレイサーバを以下のように選択してください．

- `Ubuntu` もしくは `Ubuntu on Wayland` が選択可能な場合 → `Ubuntu`
- `Ubuntu on Xorg` もしくは `Ubuntu` が選択可能な場合 → `Ubuntu on Xorg`

## 明らかに存在するプロジェクトフォルダが存在しないと言われる: Local directory /.../hoge.TBS does not exist.

---

Python の`os.path.isdir()`が，ディレクトリが存在するにも関わらず`False`を返しています．
原因は不明ですが，apt パッケージを更新することで改善しました．

```bash
$ sudo apt update
$ sudo apt upgrade -y
$ sudo apt autoremove --purge -y
$ sudo apt autoclean
```

## FC と PC 間の ROS 通信ができない

---

### 1. ファイアウォールがUDPを弾いている可能性

ROS 2 のネットワーク間通信は内部で UDP を用いていますが，
ファイアウォールがそれを許可していない可能性があります．
以下のコマンドでファイアウォールの状態を確認し，
許可されたポートリストに UDP の 7400 台のポートが含まれなければこれが原因かもしれません．

```bash
$ sudo ufw status
```

本当は使用するポートのみを許可するのが望ましいですが，
ひとまずUFWを無効化して再起動すると通信できるようになります．

```bash
$ sudo ufw disable
$ sudo reboot
```

## ユーザコードを作成して書き込んだが FC が動作していない

---

ランタイムエラーが発生している可能性があります．
SSH でラズパイにログインし，`journalctl`でコンソール出力を確認すると手がかりが掴めるかもしれません．
十字キーで移動し，`Q`キーで終了できます．

```bash
$ ssh pi@${hostname}.local  # or pi@${ip_address}
$ journalctl -u tobas_real_realtime.service -e  # or tobas_real_interface.service
```
