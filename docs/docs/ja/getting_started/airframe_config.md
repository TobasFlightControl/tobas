# 機体構成の設定

<!-- ゲームの広告と同じで，全てを理解することよりもとりあえず何も考えずに簡単に動かせることが大事． -->
<!-- 後々必要になる面倒な作業は隠して面白いところを見せる． -->

Tobas Setup Assistant を用いて機体構成に関する設定を行います．
Tobas Setup Assistant は，Tobas を用いてドローンを飛ばすのに必要なプロジェクトフォルダを作成するための GUI です．
プロジェクトフォルダには，機体の質量特性，プロペラの空気特性，モータの電気特性といった，ドローンを飛ばすのに必要な全ての情報が含まれます．
Tobas Setup Assistant を使用するためには，ユーザの機体を表現した Universal Aircraft Description Language (UADF) ファイルが必要です．
UADF の詳細については[What is UADF](../additional_information/what_is_uadf.md)をご参照ください．

## 準備

---

このチュートリアルでは，典型的なクアッドコプターである DJI F450 を使用します．
構成部品は以下のとおりです：

- フライトコントローラ: <a href=https://tobas.jp/product target="_blank">Tobas FC101</a>
- Power Module: <a href=https://tobas.jp/product target="_blank">Tobas PM101</a>
- フレーム: <a href=https://ja.aliexpress.com/item/1005007683004849.html target="_blank">DJI F450 Frame</a>
- バッテリー: <a href=https://ja.aliexpress.com/item/4000244479545.html target="_blank">HRB 3S 5000mAh 50C</a>
- モータ: <a href=https://ja.aliexpress.com/item/1005008178619191.html target="_blank">DJI A2212 920KV</a> (CW x 2, CCW x 2)
- プロペラ: <a href=https://ja.aliexpress.com/item/1005004372872772.html target="_blank">DJI 9450</a> (CW x 2, CCW x 2)
- ESC: <a href=https://www.fly-color.net/index.php?c=category&id=234 target="\_blank">Flycolor Raptor5 35A</a> x 4
- GNSS アンテナ: <a href=https://www.topgnss.store/en-jp/products/2pcs-l1-l5-helical-antenna-uav-flight-control-antenna-gps-glonass-galileo-bds-rtk-handheld-receiver-an-103-topgnss-helical target="_blank">TOPGNSS AN-103</a>
- RC 受信機: <a href=https://www.rc.futaba.co.jp/products/detail/I00000018 target="_blank">Futaba R3001SB</a>

機体の UADF を作る必要がありますが，今回は既に用意してあるものを使用します．

## 起動

---

アプリケーションメニューから`TobasSetupAssistant`を起動するか，もしくはターミナルで以下を実行します．

```bash
$ ros2 launch tobas_setup_assistant setup_assistant.launch.py
```

![start](../../assets/airframe_config/start.png)

## UADF の読み込み

---

`New`をクリックし，ファイルダイアログで`/opt/tobas/share/tobas_description/urdf/f450.uadf`を選択して`Open`をクリックします．
すると，モデルビューに機体が表示され，各設定ページが有効になります．

![load](../../assets/airframe_config/load.png)

## Propulsion System

---

推進系の設定を行います．
今回は電動機のため`Electric Propulsion System`にチェックを入れたままにします．

### Battery

バッテリーの設定を行います．
バッテリーの仕様を確認し，各項目に適切な値を入力してください．

![battery](../../assets/airframe_config/propulsion/battery.png)

### Propulsion Units

各推進ユニットの設定を行います．

まず`propeller_0`リンクの設定を行います．
各部品の仕様を確認し，`ESC`，`Motor`，`Propeller`の各項目に適切な値を入力してください．

<!-- prettier-ignore-start -->
!!! tip
    機体上のプロペラと表示されているリンク名との対応関係がわからない場合は，
    画面左上の`Frame Tree`からリンク名をクリックすることでモデルビューにハイライトされます．
<!-- prettier-ignore-end -->

![propulsion/esc](../../assets/airframe_config/propulsion/esc.png)

![propulsion/motor](../../assets/airframe_config/propulsion/motor.png)

![propulsion/propeller](../../assets/airframe_config/propulsion/propeller.png)

`Aerodynamics`ではプロペラの空力特性の設定を行います．
複数の設定方法から選ぶことができますが，今回使用するプロペラである DJI 9450 は予めモデルが用意されているので，それを使います．
最初の選択リストから`Select Propeller Model`を選択し，その下の選択リストで`dji_9450`を選択してください．

![propulsion/aerodynamics](../../assets/airframe_config/propulsion/aerodynamics.png)

この機体の 4 つの推進ユニットは全て同一なため，`Copy To All`をクリックし，`propeller_0`の設定を他の 3 つにコピーします．
`propeller_0`の設定が他のタブにも反映されていることを確認してください．

## Hardware Interface

---

ハードウェアの接続に関する設定を行います．　
`Tobas FC1xx`が選択されていることを確認し，
4 つの推進ユニットそれぞれについて，適切に DShot チャンネルを設定してください．

![hardware_interface](../../assets/airframe_config/hardware_interface.png)

<!-- prettier-ignore-start -->
!!! note
    ここでインターフェースを指定しない場合，例えば CAN-ESC を使う場合などは，
    そのままではハードウェアは駆動されないため，
    ハードウェアと Tobas ソフトウェアを繋ぐ ROS ノードを自作する必要があります．
<!-- prettier-ignore-end -->

## Remote Connection

---

地上局から FC に遠隔で接続するための設定を行います．

### Network Interface

FCが外部と通信する際に使用するネットワークインターフェースを指定します．
今回はFCとGCSの通信にポケットWi-Fiを使うため，`Wireless`を選択します．
有線LANを使用する場合は`Wired`，
外部の通信モジュールを使わずラズパイ内臓のアクセスポイントを使用する場合は`Access Point`を選択してください．
これら以外の構成場合，例えばVPNのインターフェース`tun0`を使う場合などは，`Other`を選択し直接インターフェース名を指定してください．

### Host

GCSがLAN内でFCを識別するためのアドレスの設定を行います．
FCのIPを固定していればそれを直接使用することもできますが，今回は手軽なホスト名を使用することにします．
`Hostname`を選択し，[Boot Device Configuration](./bootmedia_config.md)で設定したFCのホスト名を入力してください．

![remote_connection](../../assets/airframe_config/remote_connection.png)

## プロジェクトの保存

`Save`をクリックし，ファイルダイアログで`~/.local/share/tobas/colcon_ws/src/`以下に`tobas_f450.TBS`として保存します．

![save](../../assets/airframe_config/save.png)

## 次の手順へ

---

これで作業は完了です．
Setup Assistant を閉じてください．
次は，実機を作成してハードウェアのセットアップを行います．
