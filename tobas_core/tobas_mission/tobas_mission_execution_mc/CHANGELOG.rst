^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_mission_execution_mc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* geographic関連をクラス化
* GNSS関連の計算をgeographilibに
* .
* サービスコールの制限時間を無制限に
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* .
* wip
* Merge branch 'feature/spline_waypoint' into jazzy
* CatmullRomPathを共通化
* pause -> stop_at_waypoint
* .
* 水平成分と鉛直成分の比率によって制約を変化させる
* ウェイポイントスプラインコマンドを追加
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* .
* fix: 停止動作を改善
* time_optimal -> jerk_limited
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* 海抜高度で指定可能に
* 表示上のコマンド番号を1-indexedに
* 最後に実行中だったミッション番号を返す
* TakeoffでPre-Arm Checkを確認
* .
* ミッションの実行可能性をチェック
* NaNならばデフォルト値を使う
* tobas_mission_msgs/action/ExecuteMissionの型変更に対応
* Merge branch 'jazzy-dev' into jazzy
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* .
* Merge branch 'jazzy-dev' into jazzy
* .
* fix
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* ミッションコマンド開始前に一時停止
* 制御器の設定値を引き継ぐ
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* move
* ミッションキャンセルや上書きの際は一度滑らかに停止する
* add compile option: -Wswitch-enum
* 加速度指令を追加
* PosVelコマンドにAccを追加
* .
* 着陸検知の基準を変更
* .
* fix
* ミッションの目標位置を連続に
* tobas_constantsをファイル分割
* ミッションの高度はアーム地点を基準にする
* heading_rate, heading_accelをパラメータに
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* change: CommandLevel -> Priority
* change: ミッションに優先度を定義
* ミッションにエラーコードを追加
* .
* 手動操縦が有効だったらミッションが実行できないように
* .
* ミッションの実行中に新しいのが来たら上書き
* change: RTLでLandまでやる
* Mission Executorの設定画面を追加
* プロジェクトのデフォルトパラメータを使用可能に
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* Landでは直前のコマンドの経緯度を引き継ぐ
* 方位の軌跡を緩やかに計画
* Merge branch 'jazzy-dev' into jazzy
* unique_ptr -> shared_ptr
* .
* アクションの引数をconst参照に
* .
* fix
* Merge branch 'feature/mission_execution' into jazzy-dev
* PX4のRTL45度逆円錐ルール
* modify
* add auto_heading option
* アクションクライアントを対応
* .
* ミッション実行用のノードを1箇所に統合
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------

2.8.0 (2025-09-25)
------------------

2.7.0 (2025-09-09)
------------------
