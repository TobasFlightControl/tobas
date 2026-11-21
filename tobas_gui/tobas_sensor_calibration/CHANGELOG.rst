^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_sensor_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.10 (2026-11-21)
--------------------
* Merge branch 'feature/refine-gui-layout' into jazzy
* remove: Drop the legacy body font size constant
* fix: Remove remaining legacy layout constraints
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
* modify: Finish remaining GUI layout backport
  Replace legacy parameter getters with shared forms and standalone
  compound editors. Add individual scrolling, align descriptions and
  controls, and apply the remaining spacing and calibration drawing changes.
  Keep existing saved-setting keys and numeric precision. Use standard Qt
  setters for numeric inputs.
  Validation: GUI package builds and pre-commit passed. Saved-setting
  snapshots matched 168 baseline entries; normal and high-DPI GUI checks
  passed.
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
* Merge branch 'feature/use-geographiclib' into jazzy
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* geographic関連をクラス化
* GNSS関連の計算をgeographilibに
* .
* .
* コメント改善
* サービスコールの制限時間を無制限に
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* add license
* add license
* .
* .
* ドローンの名前空間を絶対パスで指定
* .
* 閾値を増やした
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* .
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* アームされたらキャリブレーションをキャンセル
* アームされたらキャリブレーションをキャンセル
* .
* .
* タイムアウトを延長
* 地磁気キャリブレーション後に再起動を促す
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into release/v2.12
* fix
* Merge branch 'jazzy-dev' into jazzy
* modify: 時間をstd::chrono::durationで管理
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: qDebug
* 不要な定義を削除
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* .
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: fabs -> std::abs
* 別スレッドの実行を簡略化
* SshClientのQtラッパーを実装して使用
* Merge branch 'jazzy-dev' into jazzy
* tobas_std -> tbs
* 加速度キャリブレーションの閾値を緩めた
* change: 世界座標系をNWU座標系からENU座標系に変更
* change: S.BUSメッセージにframe_lostなどを追加
* change: RCスイッチのチャンネル変更
* change: Futabaに合わせてスロットルをリバース前提に
* modify: get_clock()->now() -> now()
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* 依存解決
* キャリブレーションの開始時にトピックチェック
* .
* .
* Implement "Large Vehicle Calibration"
* .
* .
* rename
* 地磁気センサの手法を選択式に
* edit cmakes
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: gui::common -> gui::cmn
* modify: 典型的な文字サイズをtobas_gui_commonにまとめた
* .
* modify: キャリブレーションが済んでいることをタブの色で表示
* .
* wip: キャリブレーション済みかどうか表示
* .
* change: Hardware SetupをSensor CalibrationとActuator Testに分離
* Contributors: Masayoshi Dohi
