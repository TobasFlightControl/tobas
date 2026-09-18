^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_ic_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* Add description
* 0.0.0 -> 2.16.0
* .
* コメント改善
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* wip
* wip
* clang-format
* Merge pull request `#11 <https://github.com/TobasFlightControl/tobas/issues/11>`_ from TobasFlightControl/feature/add_cxgb400_functions
  Feature/add cxgb400 functions
* fix: you cannnot set aperture to cxgb400
* chore
* feat: add functions related to exposure status settings
* Contributors: Masayoshi Dohi, ok10171

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* NAV_SATのデコーダを実装
* enum struct -> enum class
* C由来の関数を排除
* C由来の関数を排除
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* .
* modify: 前置インクリメントを使用
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* .
* Merge branch 'jazzy-dev' into jazzy
* Enable BDU
* .
* Enable BDU
* 地磁気の軸をROSノード側で入れ替える
* .
* .
* Merge pull request `#8 <https://github.com/TobasFlightControl/tobas/issues/8>`_ from TobasFlightControl/feature/bmm350_driver
  add: Implement BMM350 driver functions and nodes
* move drivers: ILPS22QS, IIS2MDC
* fix: BMM350 review comments
* add: Implement BMM350 driver functions and nodes
* 加速度のアンチエイリアシング
* fix: 別スレッドの終了処理
* delete: nssnmfg_pefc
* add license
* add license
* refactor: ZED-F9P driver
* .
* Merge branch 'feature/add_namespace' into jazzy-dev
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* .
* Merge branch 'jazzy-dev' into jazzy
* add compile option: -Wswitch-enum
* Merge pull request `#4 <https://github.com/TobasFlightControl/tobas/issues/4>`_ from TobasFlightControl/feature/refine_img_processing
  Feature/refine img processing
* fix: code style
* feat: moved cxgb_400 driver variables to public space
* feat: camera ROS driver services
* fix: camera driver getCameraStatus function, feat: publish camera status
* add: tobas_camera_msgs
* feat: add commands to cx_gb400 driver
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171, onionitsuka

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* wait before each samplings
* fix
* IMUのaccel, gyroを同時に読み取る
* .
* Merge branch 'jazzy-dev' into jazzy
* refactor
* refactor
* refactor: 岡本くんのコミット全般
* pre-commit run -a
* Merge pull request `#2 <https://github.com/TobasFlightControl/tobas/issues/2>`_ from TobasFlightControl/feature/camera_driver_rebase
  Feature/camera driver rebase
* fix: tobas_ic_drivers/cx_gb400 style
* fix: tobas_linux/video_dev style
* add: tobas_ic_drivers cx_gb400 driver
* tobas_std -> tbs
* modify: std::thread -> std::jthread
* .
* Contributors: Masayoshi Dohi, ok10171

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: zed_f9p.cpp
* modify: namespace ch = std::chrono;
* .
* コンパイルオプションを追加
* refactor
* Contributors: Masayoshi Dohi
