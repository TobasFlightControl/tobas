^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_gazebo_system_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.5 (2026-09-26)
-------------------

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
* fix
* 0.0.0 -> 2.16.0
* fix message qos
* Merge branch 'feature/use-geographiclib' into jazzy
* Add geoid unduration
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* geographic関連をクラス化
* GNSS関連の計算をgeographilibに
* .
* コメント改善
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* English translation
* .
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* フィルタの設定の返り値を変更: bool -> void
* RPMフィルタのLPFカットオフを調整可能に
* RPMフィルタのパラメータを調整可能に
* ConfigureImuFilter -> ConfigureImuLowPassFilter
* rename: LowPassFilterP1 -> LowPassFilter
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* memcpy -> std::memcpy
* Merge branch 'jazzy-dev' into jazzy
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* disable type description services
* .
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* delete unecessary include
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* .
* .
* delete unecessary includes
* .
* refactor
* clear README.md
* Merge pull request `#6 <https://github.com/TobasFlightControl/tobas/issues/6>`_ from TobasFlightControl/fix/lidar_topic_name
  fix: specify lidar topic name from sdf
* fix: specify lidar topic name from sdf
* add compile option: -Wswitch-enum
* .
* delete unecessary includes
* refactor
* Merge pull request `#5 <https://github.com/TobasFlightControl/tobas/issues/5>`_ from TobasFlightControl/feat/lidar_plugin
  Feat/lidar plugin
* fix: remove imu from the generalization viewe point
* fix: code style
* add: tobas_gazebo_system_plugin README.md
* edit qos
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* add: hesai lidar plugin
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* add gazebo packages
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* modify
* odometry.statusを削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* modify: fabs -> std::abs
* Merge branch 'feature/impl_rope_plugins' into jazzy-dev
* GUIに吊り下げ荷物の着脱機能を追加
* SDFパラメータを最小限に
* ヤング率とCSAをサービスのパラメータに
* ラインIDを固定
* .
* 荷重の回転を打ち消す方向に働くトルクを追加
* プラグイン完成
* wip: plugin
* .
* wip
* .
* .
* 通常のMarkerManagerを使用
* add: TobasMarkerManager
* rename
* wip: テザーロープの端点のマーカーを発行
* rename
* wip: tether_station_force_plugin.cpp
* .
* .
* modify: ROSノードスレッドの終了処理
* Merge branch 'jazzy-dev' into jazzy
* modify: ノード名に適した形に自動修正
* tobas_std -> tbs
* change: 世界座標系をNWU座標系からENU座標系に変更
* Merge branch 'feature/sim_world_settings' into jazzy-dev
* 世界モデルから原点の経緯高度を取得
* remove: simulation page
* wip: RenderInfo
* move: tobas_gazebo
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
