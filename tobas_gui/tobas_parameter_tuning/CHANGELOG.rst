^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_parameter_tuning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.10 (2026-11-21)
--------------------
* Merge branch 'feature/refine-gui-layout' into jazzy
* fix: Complete panel scrolling and GUI sizing
  Scroll joint commands independently and remove redundant outer scrolling
  from settings and battery selection. Let tab and button heights follow
  the font, keep table columns automatic, and match legend inset and
  navigation padding to the source layouts.
  Apply the source stretch factors while preserving saved-setting keys,
  numeric precision, validation, and communication behavior.
  Validation: 14 GUI packages built, pre-commit and normal/high-DPI GUI
  checks passed, and 168 saved-setting entries matched the baseline.
* fix: Align backported GUI layouts and includes
  Match the source layouts for robot property spacing, flight log panel
  proportions, command margins, project path width, and settings titles.
  Size plot selectors to their contents to avoid clipped text.
  Group and alphabetize includes according to the C++ coding rules.
  Validation: 14 GUI package builds and pre-commit passed. Normal and
  high-DPI GUI checks passed; 168 saved-setting entries matched the
  baseline.
* modify: Backport adaptive GUI layouts to Jazzy
  Backport layout changes from e61bc6fe1 and 740d47e35 to the Qt 5 GUI.
  Preserve existing APIs, parameter keys, value ranges, and ROS behavior.
  Validate all 14 affected packages, pre-commit checks, settings round
  trips against the previous version, and GUI checks at 1x and 2x scale.
* Contributors: Masayoshi Dohi

2.16.9 (2026-10-19)
-------------------

2.16.8 (2026-10-16)
-------------------

2.16.7 (2026-10-14)
-------------------

2.16.6 (2026-10-03)
-------------------

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
* 動的パラメータサービスにエラーメッセージを追加
* .
* サービスコールの制限時間を無制限に
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* .
* fix
* Merge branch 'jazzy-dev' into jazzy
* add license
* add license
* ドローンの名前空間を絶対パスで指定
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
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

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* 依存解決
* edit cmakes
* .
* modify: blockSignals -> QSignalBlocker
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: gui::common -> gui::cmn
* rename: tobas_parameter_tuning_gui -> tobas_parameter_tuning
* Contributors: Masayoshi Dohi
