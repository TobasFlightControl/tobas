^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_msgs_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* 0.0.0 -> 2.16.0
* Merge branch 'feature/use-geographiclib' into jazzy
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* GNSS関連の計算をgeographilibに
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* GNSSメッセージに衛生数を追加 + HP位置を使用しない
* Merge branch 'jazzy-dev' into jazzy
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
* Merge branch 'feature/add_namespace' into jazzy-dev
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* add method: setNaN()
* add odometry messages
* tobas_object_avoidance interface
* UniquePtr, ConstUniquePtrを定義
* Merge branch 'release/v2.12' into feature/refine_img_processing
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
* Merge branch 'feature/mission_execution' into jazzy-dev
* odometry.statusを削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* change: RCスイッチのチャンネル変更
* change: 加速度から振動レベルを計算
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* refactor
* Contributors: Masayoshi Dohi
