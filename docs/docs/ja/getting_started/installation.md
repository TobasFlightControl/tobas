# インストール

## PC への Tobas のインストール

---

### ROS 2 Jazzy のインストール

<a href=https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html target="_blank">ROS 2 Jazzy の公式手順</a>
に従って ROS 2 を deb パッケージからインストールしてください．
このドキュメントでは`ros-jazzy-desktop`を使用します．

### Tobas のインストール

ROS 2 の apt リポジトリから Tobas をインストールします．

```bash
$ sudo apt update
$ sudo apt install ros-jazzy-tobas
```

Tobas を使用するターミナルでは，ROS 2 Jazzy の環境を読み込んでください．

```bash
$ source /opt/ros/jazzy/setup.bash
```

## フライトコントローラのイメージ書き込み

---

### 必要なもの

- <a href=https://www.raspberrypi.com/products/raspberry-pi-5/ target="_blank">Raspberry Pi 5 (2 GB以上)</a>
- Tobas HAT <!-- TODO: URL -->
- 16 GB 以上のマイクロ SD カード (例: <a href=https://www.sandisk.com/products/memory-cards/microsd-cards/sandisk-extreme-uhs-i-microsd target="_blank">SanDisk Extreme microSDXC™ UHS-I CARD - 32GB</a>)
- SD カードリーダー (例: <a href=https://www.sandisk.com/products/accessories/memory-card-readers/sandisk-quickflow-microsd-memory-card-reader-usb-c target="_blank">SANDISK QuickFlow microSD Card Reader with USB-C</a>)

### 手順

<a href=https://drive.google.com/file/d/1ECkhHWGlVKN-8F47updfDKAXjJOI3xxc/view target="_blank">tobas_2.15.0_arm64.img.gz</a>
をダウンロードしてください．

適当なイメージフラッシャーをインストールしてください．例えば以下が使用可能です．

- <a href=https://etcher.balena.io/ target="_blank">balenaEtcher</a>
- <a href=https://www.raspberrypi.com/software/ target="_blank">Raspberry Pi Imager</a>

SD カードを，カードリーダーを介して PC に接続してください．

イメージフラッシャーを起動し，ダウンロードしたイメージと対象の SD カードを選択し，書き込みを開始してください．
以下は balenaEtcher の画面です．

![balena_etcher](../../assets/installation/balena_etcher.png)

正常に終了したら SD カードを PC から取り外してください．

## 次の手順へ

---

これでインストールは完了です．
インストール時に実行された変更を反映するため，次の操作に進む前に一度 PC を再起動しておくことをおすすめします．
次は Tobas Bootmedia Config を用いて起動前の初期設定を行います．
