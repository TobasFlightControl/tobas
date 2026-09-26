^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_drone_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* 0.0.0 -> 2.16.0
* コメント改善
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* .
* English translation
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* change: 効率を考慮したミキシング
* enum struct -> enum class
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* .
* 中心ピッチ角を予め指定した値にする．
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* Merge branch 'jazzy-dev' into jazzy
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
* .
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* yaml double format
* modify: configurations -> configuration
* Merge branch 'jazzy-dev' into jazzy
* delete: JointRole::kManipulation
* tobas_std -> tbs
* change: バッテリーの内部抵抗を補償した電圧で判定
* change: 世界座標系をNWU座標系からENU座標系に変更
* change: 追加ジョイントの仕様
  UserActive, UserPassiveタイプを追加
  LandingGear, PassiveWheel, Otherタイプを削除
  JointCommanderでは位置，速度，力をそのまま発行
* change: 可変ピッチプロペラの反トルク係数をピッチ角の関数に変更
* wip: 可変ピッチプロペラの反トルク係数を他変数に
* fix: computeEngineSpeed
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* fix: ロータオブジェクト取得でNULLチェック
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* .
* コンパイルオプションを追加
* .
* refactor
* Contributors: Masayoshi Dohi
