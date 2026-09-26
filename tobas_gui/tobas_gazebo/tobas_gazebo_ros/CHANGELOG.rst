^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_gazebo_ros
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
* Add description
* 0.0.0 -> 2.16.0
* .
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
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
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* .
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Contributors: Masayoshi Dohi, Tetsuka Daiki

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* move: tobas_gazebo
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
