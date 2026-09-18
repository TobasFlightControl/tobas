^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_components_rt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* Add description
* 0.0.0 -> 2.16.0
* .
* .
* .
* load, unloadを排他処理にする必要はなし
* 同じタイマーイベントは高々1つだけキューに入れる
* ExecutorをSingleThreadedExecutorからrclcpp::experimental::executors::EventsExecutorに変更
* English translation
* Merge branch 'feature/english_translation' into jazzy
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/refactor_rviz_plugin' into jazzy-dev
* refactor
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* tobas_components_rt
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* disable type description services
* .
* fix node options
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* add compile option: -Wswitch-enum
* Merge branch 'release/v2.12' into feature/refine_img_processing
* change: リアルタイム優先度を下げた
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into release/v2.12
* fix
* Merge branch 'jazzy-dev' into jazzy
* .
* .
* .
* .
* .
* refactor
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* .
* コンパイルオプションを追加
* refactor
* Contributors: Masayoshi Dohi
