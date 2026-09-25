^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_y_axis_tilt_multi_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add description
* 0.0.0 -> 2.16.0
* Merge branch 'feature/consider-fixed-thrust-joint' into jazzy
* 静的状態と動的状態を分離
* .
* .
* 固定ロータを含む場合に対応
* チルト軸のオフセットを毎周期更新
* .
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* English translation
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* 位置制御の自然周波数と減衰比のデフォルト値を変更
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* Merge branch 'jazzy-dev' into jazzy
* コマンドの動的メモリ確保を減らした
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* 制御しない設定値にはNaNを入れる
* 設定値のうち制御しない値は現在値を入れる
* 制御器フィードバックメッセージの内容を減らしてSetpointに移す
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* add compile option: -Wswitch-enum
* 加速度指令を追加
* PosVelコマンドにAccを追加
* .
* 積分制御のアンチワインドアップを実装
* 動的パラメータのスケール変更など
* .
* 位置制御のゲインを推力に応じてスロットル
* .
* fix
* shared_ptr -> unique_ptr
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* 垂直方向の最大加速度を増加
* change: CommandLevel -> Priority
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix
* fix: 推力が低いときのチルト角を安定化
* fix
* Y軸チルトコプターのレートコマンドを追加
* Merge branch 'jazzy-dev' into jazzy
* 積分誤差のリセットは着陸県知事ではなくディスアーム時に
* Merge branch 'feature/mission_execution' into jazzy-dev
* .
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* change: 着陸検知での自動ディスアームを廃止
* modify: fabs -> std::abs
* Merge branch 'jazzy-dev' into jazzy
* modify: 鉛直上向きに力を出すことを保証
* modify: I成分はゼロにはできないように
* move: tobas_std_tools/float.hpp -> tobas_math/float.hpp
* tobas_std -> tbs
* change: 着陸したら強制的にディスアーム（アイドリングストップ）
* change: 可変ピッチプロペラの反トルク係数をピッチ角の関数に変更
* modify: get_clock()->now() -> now()
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* リンクエラーを検出
* .
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify
* modify: ミキサーをそれぞれのコントローラパッケージに移動
* rename: MultiRotorControllerFeedback -> MulticopterControllerFeedback
* fix: mixer.xpp
* .
* fix: tobas_y_axis_tilt_multi_controller
* fix: mixer
* fix package name
* fix: チルト角のオフセットと符号を考慮
* .
* コンパイルオプションを追加
* rename: x_axis_tilt_multicopter -> y_axis_tilt_multicopter
* Contributors: Masayoshi Dohi
