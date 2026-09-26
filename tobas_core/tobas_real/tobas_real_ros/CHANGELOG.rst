^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_real_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.5 (2026-09-26)
-------------------

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
* magic_enum.hpp -> magic_enum/magic_enum.hpp
* magic_enumとcxxoptsのサブモジュールを削除
* 0.0.0 -> 2.16.0
* .
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* wip
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* フィルタの設定の返り値を変更: bool -> void
* rename: LowPassFilterP1 -> LowPassFilter
* TOBAS_CHECK -> TOBAS_ASSERT
* Merge branch 'jazzy-dev' into jazzy
* 中心ピッチ角を予め指定した値にする．
* CPUの温度と負荷を平滑化
* add license
* add license
* fix topic
* .
* fix: use real/ topic
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* ジャイロバイアスの計測中に動いたら3秒待機
* .
* add compile option: -Wswitch-enum
* .
* tobas_constantsをファイル分割
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
* センサの更新確認を削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: abs -> std::abs
* warn -> info
* modify: configurations -> configuration
* Merge branch 'jazzy-dev' into jazzy
* modify: size() > 0 -> !empty()
* modify: CPUの読み取り周波数を変更: 1Hz -> 10Hz
* tobas_std -> tbs
* modify: 機体動作判定用のジャイロをLPFに通す
* .
* change: S.BUSメッセージにframe_lostなどを追加
* change: RCスイッチのチャンネル変更
* change: Futabaに合わせてスロットルをリバース前提に
* change: 可変ピッチプロペラの反トルク係数をピッチ角の関数に変更
* modify: move -> std::move
* modify: get_clock()->now() -> now()
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* リンクエラーを検出
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* fix: add dependency
* .
* コンパイルオプションを追加
* refactor
* Contributors: Masayoshi Dohi
