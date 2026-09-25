^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_fc1xx_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.4 (2026-09-25)
-------------------

2.16.3 (2026-09-25)
-------------------
* fix: Backport remaining Jenkins build fixes
  Backport the six fix/jenkin-build commits from 978fce779 through
  90e093ae3, merged into rolling by 6042bef53.
  Declare Eigen, Qt and Boost dependencies, remove the unused OpenBLAS
  and Boost array dependencies, and replace Boost polymorphic casts
  with standard C++ casts.
  Preserve Jazzy-specific code and apply the property-tree change at
  its existing package location.
* Contributors: Masayoshi Dohi

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
* Merge branch 'feature/use-geographiclib' into jazzy
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* GNSS関連の計算をgeographilibに
* .
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* GNSSメッセージに衛生数を追加 + HP位置を使用しない
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* fix
* done
* Rotor Control Gain -> RPM Control Gain
* Merge branch 'jazzy-dev' into jazzy
* フィルタの設定の返り値を変更: bool -> void
* IMUのサンプリング周波数を全体で固定
* ConfigureImuFilter -> ConfigureImuLowPassFilter
* rename: LowPassFilterP1 -> LowPassFilter
* .
* .
* fix: 負の電流を許容
* Merge branch 'jazzy-dev' into jazzy
* 地磁気の軸をROSノード側で入れ替える
* Merge branch 'feature/fc2xx_driver' into jazzy-dev
* .
* move drivers: ILPS22QS, IIS2MDC
* .
* IMU 800Hz
* 加速度のスケール変更
* add license
* add license
* fix: use real/ topic
* refactor: ZED-F9P driver
* .
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Add success flag
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* modify
* コンパスの更新レートを変更: 50Hz -> 100Hz
* センサの更新確認を削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: モータコントローラとの通信に失敗してもモータの状態を発行
* .
* .
* warn -> info
* IMUのaccel, gyroを同時に読み取る
* Merge branch 'jazzy-dev' into jazzy
* clang-format
* change: T1 -> FC1xx
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
