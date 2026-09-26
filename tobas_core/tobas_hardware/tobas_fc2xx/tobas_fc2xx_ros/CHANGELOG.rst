^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_fc2xx_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.5 (2026-09-26)
-------------------
* Add dependencies
* Contributors: Masayoshi Dohi

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
* change initialization interval: 1sec -> 3sec
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
* done
* Rotor Control Gain -> RPM Control Gain
* Merge branch 'jazzy-dev' into jazzy
* IMUのサンプリング周波数を全体で固定
* RPMフィルタのLPFカットオフを調整可能に
* RPMフィルタのパラメータを調整可能に
* ConfigureImuFilter -> ConfigureImuLowPassFilter
* DShotドライバの初期化を成功するまでやり直すようにする
* .
* fix: 負の電流を許容
* Merge branch 'jazzy-dev' into jazzy
* 軸入れ替え
* fix
* fix
* Merge branch 'feature/fc2xx_driver' into jazzy-dev
* fix
* add: hardware_interfaces.launch.py
* add: pwm_batt_imu_driver
* .
* add: dshot_driver.cpp
* .
* add driver nodes
* Contributors: Masayoshi Dohi

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------

2.10.0 (2026-01-29)
-------------------

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
