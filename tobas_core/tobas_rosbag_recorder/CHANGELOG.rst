^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_rosbag_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* Add description
* 0.0.0 -> 2.16.0
* .
* Merge branch 'feature/gcs_layout' into jazzy
* delete unecessary "this->"
* Merge branch 'feature/english_translation' into jazzy
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: 分割コンパイル
* modify: 分割コンパイル
* 実機かSIMかでログ取得後に追加するリストを切り替える
* Merge branch 'jazzy-dev' into jazzy
* ディスクの空き容量が少なくなったらログを停止
* Merge branch 'jazzy-dev' into jazzy
* .
* 10GB制限を撤廃（どうせWriter::openでこける）
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* 制御器フィードバックメッセージの内容を減らしてSetpointに移す
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* add compile option: -Wswitch-enum
* edit qos
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* change: CommandLevel -> Priority
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* Merge branch 'jazzy-dev' into jazzy
* modify: move -> std::move
* modify: get_clock()->now() -> now()
* change: Pre-Arm Check, Post-Arm CheckをVehicle Healthにまとめた
* change: 振動レベルのログを表示
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* add dependency
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: パス作成時のエラーハンドリング
* rename: MultiRotorControllerFeedback -> MulticopterControllerFeedback
* .
* コンパイルオプションを追加
* メッセージを移動など
* メッセージを移動など
* refactor
* Contributors: Masayoshi Dohi
