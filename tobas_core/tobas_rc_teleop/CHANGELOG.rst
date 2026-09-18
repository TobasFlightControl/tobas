^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_rc_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* Add description
* magic_enum.hpp -> magic_enum/magic_enum.hpp
* magic_enumとcxxoptsのサブモジュールを削除
* 0.0.0 -> 2.16.0
* .
* コメント改善
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* ロイターモードにFF加速度を追加
* 2次系でジャークを出力して参照軌道を決める
* wip: FF加速度を指令
* .
* アームポジションの閾値を動的パラメータに
* arm/disarm durationを静的パラメータに
* アーム・ディスアームのメッセージをわかりやすく
* Merge branch 'jazzy-dev' into jazzy
* TOBAS_CHECK -> TOBAS_ASSERT
* Merge branch 'jazzy-dev' into jazzy
* ゼロスロットル判定にヒステリシスを設ける
* change: パラメータのスケールとデフォルトを変更
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* .
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* modify
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* change namespace
* 鉛直速度の変化率に制約を設ける
* .
* 初期コマンドを設定値で初期化
* .
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* Killスイッチがオンのままマニュアルモードに移行できないように
* move: online_trajectory_generation
* 速度コマンドの変化率を制限
* デッドバンドを広げた
* fix
* 動的パラメータを全てdoubleにしてスケールを設定
* add compile option: -Wswitch-enum
* .
* add: thrust_angle_expo
* 着陸時に強い位置制御が入らないように
* コマンドの変化率に制約を加えた
* change default yawrate limit
* 加速度指令を追加
* PosVelコマンドにAccを追加
* .
* fix
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* 垂直方向の最大加速度を増加
* modify: 時間をstd::chrono::durationで管理
* change: CommandLevel -> Priority
* Sbus, RCInputは常に発行されるように
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix
* deadbandを変更
* Y軸チルトマルチのコマンドのプロポインターフェースを追加
* スロットルレバーのdeadbandを廃止 + 角度にはdeadbandをつける
* .
* Merge branch 'jazzy-dev' into jazzy
* modify: モード変更の判定にvehicke_healthを用いる
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: fabs -> std::abs
* modify: abs -> std::abs
* Merge branch 'jazzy-dev' into jazzy
* tobas_std -> tbs
* fix: RCの通信が死んだらコマンドをリセット
* .
* change: 陸上でゼロスロットルならば最小回転数まで落とす
* change: デフォルトパラメータ
* change: Pre-Arm Check, Post-Arm CheckをVehicle Healthにまとめた
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* 依存解決
* edit cmakes
* リンクエラーを検出
* change: パラメータ変更
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* .
* modify: 不要なインストール，エクスポートを削除
* .
* コンパイルオプションを追加
* メッセージを移動など
* Add commands for XAxisTiltMulticopter
* .
* refactor
* Contributors: Masayoshi Dohi
