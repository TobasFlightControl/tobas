^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_gcs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.4 (2026-09-25)
-------------------

2.16.3 (2026-09-25)
-------------------

2.16.2 (2026-09-22)
-------------------
* revert version
* update CHANGELOG.rst
* modify: Bump version to 2.16.2
* Contributors: Masayoshi Dohi

2.16.1 (2026-09-18)
-------------------
* v2.16.1
* Contributors: Masayoshi Dohi

2.16.0 (2026-07-29)
-------------------
* FCのバージョンがGCSよりも古ければ更新を促す．
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* .
* Add description
* 0.0.0 -> 2.16.0
* NIF -> NIC
* fix: "lo"のマルチキャストを明示
* Set redundant networking
* Set network I/F priority
* change: ネットワークの設定変更
* .
* .
* remote connectionのタイムアウトを変更: 5s -> 10s
* 予期せぬ通信断絶時にダイアログを表示
* modify
* .
* Merge branch 'feature/english_translation' into jazzy
* English translation
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* CycloneDDSでプロセス間通信を共有メモリにする設定（結局使用せず）
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* done
* Merge branch 'jazzy-dev' into jazzy
* fix: シミュレーション終了時の通信切断ではリセットしない
* add license
* add license
* UUID名前空間の中に入れて競合しないように
* ドローンの名前空間を絶対パスで指定
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* QDebugに着色
* .
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* delete unecessary include
* FCとの通信が失われた際のダイアログを削除
* 通信が切れることがわかっている場合は警告を出さない
* Merge branch 'feature/specify_nif' into jazzy-dev
* 環境変数でNIFを指定
* FCとの通信が突如切れたらGCSをリセット
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* .
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* コマンドライン引数に
* 自前のタイルサーバを立ち上げ
* add: tobas_tile_proxy
* remove unnecessary installation
* chronoに対応
* 完全にラズパイがシャットダウンされるまで適当に待つ
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* 不要な定義を削除
* property_treeのセクションを細分化
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* .
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* 最初はdisableでプロジェクト書き込み時にenable service
* ビルドエラーメッセージが長ければファイル出力
* CYCLONEDDS_URIでNIFを指定
* network.yamlを自動生成
* .
* change: config.env -> project.env
* .
* SshClientのQtラッパーを実装して使用
* modify: SSHClient -> SshClient
* fix: .gitは送信しない
* Merge branch 'jazzy-dev' into jazzy
* change: Follow XDG Base Directory Specification
* Merge branch 'feature/version' into jazzy-dev
* change: 書き込み時にFCのバージョンチェック
* change: SAとGCSでプロジェクトのバージョンチェック
* change: バージョンファイルを保存
* modify: GUIのタイトルにバージョンを表示
* edit svg files
* .
* fix
* modify
* modify: アイコン設定
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: プロジェクトのビルドを別スレッドで行う
* edit cmakes
* .
* modify: GCSのネットワークへの接続状態を監視
* .
* modify: Qtイベント処理で画面更新を安定化
* fix: トピック張替え後にQtイベントを全て処理
* change: シャットダウンボタンではGCSは落とさない
* modify
* change: 遠隔接続状態をGCSに表示
* fix: Add python path
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: rcutils_get_env -> ros2::getEnvなど
* modify: GCSの起動時に~/Tobas/colcon_ws/installをAMENT_PREFIX_PATHに加える
* modify: gui::common -> gui::cmn
* modify: 典型的な文字サイズをtobas_gui_commonにまとめた
* change: Hardware SetupをSensor CalibrationとActuator Testに分離
* rename: tobas_parameter_tuning_gui -> tobas_parameter_tuning
* modify: シミュレーション起動中はプロジェクトを読み込めないようにした
* modify: 不要なインストール，エクスポートを削除
* modify: Add project_paths.xpp
* .
* コンパイルオプションを追加
* Delete OS checks
* change: property_server, ssh_serverの通信範囲をLOCALONLYに
* modify: PropertyServerの名前空間を削除
* modify: プロジェクトが読み込まれるまでは電源ボタンを無効化
* wip: Remote Connection
* modify: Change namespace: gcs -> ctrl
* modify: Add getPkgShareDir()
* Merge branch 'feature/network_settings' into jazzy-develop
* modify: Remove kTitle
* .
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* .
* rename: tbs_path -> proj_path
* modify
* Merge branch 'jazzy-develop' into jazzy
* modify: enum format
* modify: UrdfParser, KdlParserを定義
* modify: UADFの解析結果をキャプチャ
* Merge branch 'feature/define_uadf' into jazzy-develop
* change: UADFのパスを変更
* wip
* .
* rename: Tobas Package -> Tobas Project
* change: ROS Packageのページを消し，親ディレクトリとプロジェクト名をSaveダイアログで入力
* rename: Tobas Pacakge -> Tobas Project
* modify: Load/Write -> Load/Write Project
* TBSプロジェクトをダブルクリックで選択できるように
* change maintainer email
* fix: Add dependencies
* change: 各アプリケーション専用のアイコンを作成
* fix: デスクトップアイコンが表示されない問題
* .
* change: URDF BuilderとSetup Assistantを別アプリに
* Contributors: Masayoshi Dohi
