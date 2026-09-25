^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_constants
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.4 (2026-09-25)
-------------------

2.16.3 (2026-09-25)
-------------------

2.16.2 (2026-09-22)
-------------------
* revert version
* update CHANGELOG.rst
* modify: Bump version to 2.16.2
* fix: Backport Jenkins build fixes to Jazzy
* Contributors: Masayoshi Dohi

2.16.1 (2026-09-18)
-------------------
* v2.16.1
* Contributors: Masayoshi Dohi

2.16.0 (2026-07-29)
-------------------
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* インターフェース系のパッケージのライセンスを変更
* hardcode some paths
* Add description
* 0.0.0 -> 2.16.0
* fix: "lo"のマルチキャストを明示
* change: ネットワークの設定変更
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* パラメータ管理用のサーバを実装
* Rotor Control Gain -> RPM Control Gain
* .
* enum struct -> enum class
* Merge branch 'jazzy-dev' into jazzy
* IMUのサンプリング周波数を全体で固定
* RPMフィルタのパラメータを調整可能に
* ConfigureImuFilter -> ConfigureImuLowPassFilter
* ディスクの空き容量が少なくなったらログを停止
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* ミッション保存機能を実装
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/add_custom_health_check' into jazzy-dev
* add: user-defined health statsu
* fix
  ライセンス条項を追加
  使ってない依存パッケージを削除
  コンポーネントに
  unique_ptrで発行
  トピックを相対パスに
  QoSをUDPに
  制御器側で障害物回避を制御
* Merge pull request `#7 <https://github.com/TobasFlightControl/tobas/issues/7>`_ from TobasFlightControl/feature/objetc_avoidance
  Feature/objetc avoidance
* Apply suggestion from @Masa0u0
* add license
* add license
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* 制御器フィードバックメッセージの内容を減らしてSetpointに移す
* tobas_object_avoidance interface
* add compile option: -Wswitch-enum
* .
* fix: remove imu from the generalization viewe point
* PosVelコマンドにAccを追加
* tobas_constantsをファイル分割
* カメラ系のパッケージをtobas_camera以下にまとめた
* Merge pull request `#4 <https://github.com/TobasFlightControl/tobas/issues/4>`_ from TobasFlightControl/feature/refine_img_processing
  Feature/refine img processing
* feat: register topic names to tobas_constants
* Merge branch 'release/v2.12' into feature/refine_img_processing
* add: hesai lidar plugin
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* Mission Executorの設定画面を追加
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix
* fix
* Y軸チルトマルチのコマンドのプロポインターフェースを追加
* Y軸チルトコプターのレートコマンドを追加
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* アクションクライアントを対応
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* change: コマンドが出た後なら500ms断絶で強制ディスアーム
* ビルドエラーメッセージが長ければファイル出力
* wip: Network setting page
* change: パス変更
* change: config.env -> project.env
* Merge branch 'feature/eskf_external_odometry' into jazzy-dev
* ESKF observes external pose
* Merge branch 'jazzy-dev' into jazzy
* change: Follow XDG Base Directory Specification
* modify: GUIのタイトルにバージョンを表示
* change: 地磁気の参照ベクトルを発行
* change: Pre-Arm Check, Post-Arm CheckをVehicle Healthにまとめた
* .
* change: 加速度から振動レベルを計算
* .
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* edit cmakes
* Merge branch 'feature/impl_gazebo_joint_controller' into jazzy-dev
* .
* .
* add: tobas_connection_monitor
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* .
* .
* コンパイルオプションを追加
* Add commands for XAxisTiltMulticopter
* add: tobas_x_axis_tilt_multi_controller
* refactor
* Contributors: Masayoshi Dohi
