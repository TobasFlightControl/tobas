^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas
^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* Add description
* 0.0.0 -> 2.16.0
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* PRコメント反映
* add license
* add object_avoidance on off config
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* .
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* delete: tobas_sdk
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* rename: tobas_third_party -> tobas_external
* refactor
* Merge branch 'feature/network_settings' into jazzy-develop
* move: tobas_ssh
* Merge branch 'jazzy-develop' into jazzy
* move: tobas_uadf
* add: Add UADF parser
* change maintainer email
* add: Add matrix_euler_conversion
* Merge branch 'feature/remove_sensor_covariances' into jazzy-develop
* change: tobas_config_serversで動的パラメータを管理
* change: IMU, Mag, Baroの分散を固定
* Merge branch 'jazzy-develop' into feature/mag_drivers
* remove: tobas_example_packages
* Merge branch 'jazzy-develop' into jazzy
* cmake version: 3.10 -> 3.25
* move
* rename: tobas_code_style_example -> tobas_cpp_code_style_example
* rename: tobas_coding_style_example -> tobas_code_style_example
* add: tobas_coding_style_example
* delete: tobas_launch_events
* ノードの起動順序を指定
* delete: xml version
* fix: package.xml
* add: tobas_example_packages
* .
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* impl: tobas_landing_detection
* Latency PublisherをDShot Driverに移植
* .
* move
* Merge branch 'develop'
* delete: tobas_robot_state_publisher
* tobas_teleoperationsを解体
* コマンド系のメッセージをtobas_command_msgsに移動
* Merge branch 'develop'
* PreArmCheck + PostArmCheck
* rotor_checkerを移動
* impl: tobas_disturbance_observer
* state_estimation_eskf -> tobas_eskf
* Merge branch 'develop'
* move: MatrixEulerConverter
* delete: tobas_hal, rename: hal -> real
* add: tobas_hardware
* tobas_examples -> tobas_examples_py
* Merge branch 'develop'
* Merge branch 'feature/separete_logic_and_interface' into develop
* impl: tobas_dparam
* wip: tobas_ros_interface
* Merge remote-tracking branch 'origin/feature/ros1_to_ros2' into develop
* wip: rosbag_recorder
* add: tobas_components_rt
* wip: tobas_gui
* PropertyClientでSyncServiceClientを使用
* ログレベルを設定
* タイムアウト設定
* move: tobas_rviz_plugin
* add: register.hpp
* add: tobas_gui
* wip: urdf_builder
* ros2: tobas_topic_throttle
* TypeAdapterパッケージを分離
* tobas_actions
* ros2: tobas_prearm_check
* ros2: tobas_examples
* delete: tobas_motor_test
* .
* .
* MAVROS関連を削除
* ArduPilot関連を削除
* catkin -> ament
* format="2" -> format="3"
* edit: package.xmls
* Merge branch 'develop' into main
* Merge branch 'feature/impl_hal' into develop
* モータのインターフェースをPWMではなくThrottleに統一化
* add: tobas_topic_throttle
* Merge branch 'develop' into main
* wip: property_server
* delete: tobas_constants
* Accel, Gyro, Mag, Baroのノイズ密度をリアルタイムで計測
* tobas_gazeboのパスを変更
* add: tobas_legged_robot
* Pythonのトピック名をconstants.pyに定義
* Merge remote-tracking branch 'origin/feature/ublox_debug' into develop
* .
* Merge branch 'develop' into main
* impl: tile_server
* tobas_takeoff_landig -> tobas_mission_commands
* Merge branch 'develop' into main
* .
* Merge branch 'develop' into main
* Merge branch 'feature/state_estimation_without_gps' into develop
* rename: tobas_common_actions -> tobas_pre_arm_check
* debパッケージの作成
* ディレクトリ構造を大改造
* add: tobas_fkie_master
* .
* impl: mag_calibration
* fix: package.xml
* Merge branch 'develop' into main
* Merge branch 'develop' into main
* .
* .
* add Tobas meta package
* edit: package.xml
* メタパッケージを作成
* dynamixel -> tobas_dynamixel
* Contributors: Masa0u0, Masayoshi Dohi, dohi, ok10171
