^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_eskf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* Merge branch 'feature/baro_gnss_fusion' into jazzy-dev
* .
* fix
* .
* .
* 気圧高度バイアス推定
* 気圧から高度への変換をESKF側で行う
* 気圧高度の初期値を初期データの平均で決定
* .
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* 外部Poseを受け容れるかどうかをパラメータに
* 静的パラメータのデフォルト値を削除
* アームされているならばGNSSの初期化処理を行わない
* .
* Merge branch 'jazzy-dev' into jazzy
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* permalink
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* 参照地磁気取得時の姿勢の分散の閾値を上げた
* .
* .
* .
* add compile option: -Wswitch-enum
* 地磁気の参照ベクトルの初期値を安定化
* quaternionFromEuler,eulerFromQuaternionの返り値を変更
* .
* fix
* PosVelコマンドにAccを追加
* tobas_constantsをファイル分割
* Merge branch 'release/v2.12' into feature/refine_img_processing
* change
* modify
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* modify
* odometry.statusを削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: fabs -> std::abs
* Merge branch 'feature/eskf_external_odometry' into jazzy-dev
* ESKF observes external pose
* Merge branch 'jazzy-dev' into jazzy
* .
* tobas_std -> tbs
* .
* change: 世界座標系をNWU座標系からENU座標系に変更
* .
* change: 地磁気の参照ベクトルを発行
* modify: move -> std::move
* modify: get_clock()->now() -> now()
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* edit cmakes
* リンクエラーを検出
* revert
* make static: ESKF
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: namespace ch = std::chrono;
* modify: 不要なインストール，エクスポートを削除
* .
* コンパイルオプションを追加
* refactor
* Contributors: Masayoshi Dohi
