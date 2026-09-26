^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* fix
* Add description
* 0.0.0 -> 2.16.0
* Merge branch 'feature/use-geographiclib' into jazzy
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* GNSS関連の計算をgeographilibに
* Heartbeatにシーケンス番号を追加
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* GNSSメッセージに衛生数を追加 + HP位置を使用しない
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* done
* Rotor Control Gain -> RPM Control Gain
* 実機かSIMかでログ取得後に追加するリストを切り替える
* Merge branch 'jazzy-dev' into jazzy
* RPMフィルタのLPFカットオフを調整可能に
* Quality Factorのスケールを1に変更
* RPMフィルタのパラメータを調整可能に
* ConfigureImuFilter -> ConfigureImuLowPassFilter
* ディスクの空き容量が少なくなったらログを停止
* Merge branch 'jazzy-dev' into jazzy
* CPUの温度と負荷を平滑化
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
* PRコメント反映
* add license
* add license
* add license
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* add odometry messages
* .
* modify: メッセージのコメント編集
* object avoidance draft
* .
* Merge branch 'release/v2.12' into feature/refine_img_processing
* change
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* Sbus, RCInputは常に発行されるように
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Add success flag
* Merge branch 'jazzy-dev' into jazzy
* move messages
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
