^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_failsafe
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* change: enable rotor failsafe
* .
* コメント改善
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* English translation
* Merge branch 'feature/english_translation' into jazzy
* wip
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* 離陸前にフェイルセーフが発動したらディスアーム
* 位置と速度の推定精度が低ければフェイルセーフを発動しない
* 位置推定ができているかどうかによってラジオフェイルセーフの内容を変える
* Merge branch 'jazzy-dev' into jazzy
* rename: LowPassFilterP1 -> LowPassFilter
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* fix: user_define_conditionがコケたらpre-arm check=false
* RTLに失敗したらディスアーム
* tobas_mission_msgs/action/ExecuteMissionの型変更に対応
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/add_custom_health_check' into jazzy-dev
* add: user-defined health statsu
* add license
* add license
* .
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
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* change: ミッションに優先度を定義
* ミッションにエラーコードを追加
* Sbus, RCInputは常に発行されるように
* change: RTLでLandまでやる
* プロジェクトのデフォルトパラメータを使用可能に
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* アクションクライアントを対応
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: fabs -> std::abs
* modify: abs -> std::abs
* Merge branch 'jazzy-dev' into jazzy
* add: README.md
* modify: フェイルセーフの回復処理を削除
* modify: 3次元Fixの場合のみ位置情報を更新
* tobas_std -> tbs
* .
* change: フェイルセーフで離着陸や手動操作の状態を考慮
* change: ヒステリシスを考慮して安定的にバッテリー電圧状態を切り替える
* modify: バッテリーフェイルセーフの判定条件を変更
* change: バッテリーの内部抵抗を補償した電圧で判定
* .
* change: Radio FailsafeでRTL
* .
* change: アクションで最大速度などをもとに最速軌道を生成
* wip: radio failsafe
* rename: tobas_health_monitor -> tobas_failsafe
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
