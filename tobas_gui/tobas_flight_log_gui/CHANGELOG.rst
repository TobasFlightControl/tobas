^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_flight_log_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* 依存をpackage.xmlへ
* Add description
* 0.0.0 -> 2.16.0
* Merge branch 'feature/use-geographiclib' into jazzy
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* .
* Merge branch 'feature/log_plot_scale' into jazzy
* unwrap RPY at 360 deg.
* modify
* fix
* modify
* Targetを縦軸中心に
* 最小範囲を指定
* 目標値を中心とするスケールに変更
* .
* Roll/Pitchのみスケーリング
* スケールをゼロ中心に
* modify
* 各軸のスケールを揃える
* 各モータの縦軸のスケールを揃える
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
* .
* refactor
* fix
* add: rosbagをzipで出力する機能を実装
* .
* move export thread
* レイテンシのヒストグラムを追加
* Merge branch 'feature/baro_gnss_fusion' into jazzy-dev
* ログに気圧高度バイアスを追加
* フライトログビューアに気圧を追加
* fix: list itemがnullならreturn
* .
* fix: scp_get, scp_putのファイルサイズの型を64ビット整数に
* Merge branch 'feature/scp-progress' into jazzy-dev
* ログダウンロードの進捗を表示
* クリック以外のリスト要素選択にも対応
* 実機かSIMかでログ取得後に追加するリストを切り替える
* Merge branch 'jazzy-dev' into jazzy
* IMUのサンプリング周波数を全体で固定
* .
* .
* ディスクの空き容量が少なくなったらログを停止
* ログ名の履歴を残す
* ログのダウンロードが済んだらそのままリストに追加
* .
* Merge branch 'jazzy-dev' into jazzy
* フライトログにPWMスロットルを表示
* .
* .
* 文字列結合を効率化
* .
* 更に並列化
* FFTを並列化
* フィルタリング語のFFTを追加
* .
* CPU動作周波数と温度のスケールを設定
* Export repulsive acceleration
* .
* add: RepulsiveAccelPlotWidget
* fix: clear decoder cache
* add license
* add license
* path::joinをcharに対応
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* delete unecessary include
* 無効なメッセージを表示・出力しない
* 制御器フィードバックメッセージの内容を減らしてSetpointに移す
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* .
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Y軸スケールを固定
* Merge branch 'release/v2.12' into feature/refine_img_processing
* .
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* .
* modify
* fix: データを1sec分バッファリングしてから出力
* fix: qDebug
* .
* .
* .
* .
* モータリンク名の出力とチェック
* .
* Save and reuse the last opened directory
* .
* .
* fix
* fix: CSV出力機能の修正とリファクタリング
* Merge branch 'jazzy' into jazzy-dev
* Merge pull request `#3 <https://github.com/TobasFlightControl/tobas/issues/3>`_ from TobasFlightControl:feature/export-log
  Feature/export log
* add: ログエクスポート機能の修正
  ・修正完
* add: ログエクスポート機能の修正
  ・修正完
* add: ログエクスポート機能の修正
  ・imu_raw受信から1ms以内のトピックはimu_rawと結びつけて出力するように修正
* add: ログエクスポート機能の修正
  ・出力完了後のメッセージ表示
  ・全メッセージの出力PoseからCPUまで完了
* add: ログエクスポート機能の修正
  ・rosbagの読み込み、変換、CSV出力を別スレッドで実行
* add: ログエクスポート機能の修正
  ・curDataの各要素をshared_ptrにしてデータ更新がない場合は欠損として出力
  ・シリアライズ処理にmessage_decoder.hppを使用
  ・ケース修正
* (pre-commit漏れ)
  add: ログエクスポート機能の修正
  ・Saveボタン押下後imu_rawを観測した場合CSVに1行出力
  ・他のデータは最も新しいデータを記録
  ・現状imu_rawとbatteryのみ記録
* add: ログエクスポート機能の修正
  ・Saveボタン押下後imu_rawを観測した場合CSVに1行出力
  ・他のデータは最も新しいデータを記録
  ・現状imu_rawとbatteryのみ記録
* add: ログエクスポート機能の追加（/f450/imu_rawのみ）
  フライトログをエクセル等で扱うためのrosbagをCSV出力する機能追加
  ・Ground Stationのログそれぞれに対しExportボタンを追加
  ・ボタン押下時に保存先指定ダイアログを立ち上げ
  ・Saveボタン押下後、/f450/imu_rawをタイムスタンプ通りに出力する
* Contributors: Masayoshi Dohi, Tetsuka Daiki

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: RosbagStateを受信したら有効化
* .
* 別スレッドの実行を簡略化
* SshClientのQtラッパーを実装して使用
* modify: SSHClient -> SshClient
* add rc input plot
* wip: rcin_plot
* .
* Merge branch 'jazzy-dev' into jazzy
* modify: rosbagが壊れていたら自動で修復 (reindex)
* modify: size() == 0 -> empty()
* add: CPUのログを表示
* tobas_std -> tbs
* change: ログにESC通信状態を追加
* change: 世界座標系をNWU座標系からENU座標系に変更
* .
* modify: ログビューアのレイアウトを変更
* modify: ログビューアのレイアウトを変更
* change: 振動レベルのログを表示
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* 依存解決
* edit cmakes
* modify: グリッドレイアウトを初期化
* modify: deleteでdeselect
* modify: Readで選択アイテムを回復
* .
* Joint Velocity/Effortのログを表示
* .
* グリッド内のウィジェットサイズを揃える
* clear, setTimeScaleを基底クラスに定義
* Joint Positionのログを表示
* modify: サービスにタイムアウトを設ける
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* rename: MultiRotorControllerFeedback -> MulticopterControllerFeedback
* .
* コンパイルオプションを追加
* Delete OS checks
* Merge branch 'feature/network_settings' into jazzy-develop
* delete: XXX:
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* modify
* Merge branch 'jazzy-develop' into jazzy
* modify: enum format
* change maintainer email
* Merge branch 'feature/remove_sensor_covariances' into jazzy-develop
* change: IMU, Mag, Baroの分散を固定
* fix
* change: データの共通化と不要な処理の削減によりログ表示を高速化
* modify
* fix: rotor_speed_plot.cpp
* modify: 各プロットに単位を記述
* modify: Delete using namespace std;
* modify: 枠内のウィジェットの大きさを揃える
* modify: グリッドの要素が1マスのみ専有するように
* modify: [0], at(0) -> front()
* modify
* modify
* modify: バッファを統一
* modify: imu_fft_plot.xpp
* modify: imu_fft_plot
* IMUのFFTを表示
* wip
* change: ヘッダーを削除してNew, Load, Saveボタンに
* Merge branch 'jazzy-develop' into feature/mag_drivers
* wip: Separate ROS and Qt threads
* エラーハンドリング
* Merge branch 'jazzy-develop' into jazzy
* エンジンスロットルの範囲指定
* フィルタリング前後のIMUを表示
* ログが変わったらrotor_speedとpropeller_pitchの表示をクリア
* プロット数を増やした
* .
* fix: Fix wild cards: [^.]* -> *
* cmake version: 3.10 -> 3.25
* CONFIGURE_DEPENDS
* messageのモードをSTATUSに
* sort includes
* change: include order & group
* cmake-format
* fix: Fix spelling
* clang-format
* clang-format
* clang-format
* PositionBarWidgetのテキストを綺麗に配置
* delete: xml version
* Merge branch 'main' into develop
* .
* プロペラピッチ角をログに追加
* .
* qwt::QwtPlotCurveWrapperを値で保持
* fix
* add: engine_plot
* qobject_cast -> qt::qPointerCast
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* デコード結果をキャッシュ
* edit: log_viewer.xpp
* plot accel
* 目標速度を追加
* 目標位置姿勢をプロット
* Multirotor Controllerを追加
* Merge branch 'feature/delete_euler_topic' into develop
* "euler"トピックを削除
* IMUサンプリング時間を表示
* .
* smart_ptr = nullptr -> smart_ptr.reset
* == nullptrを書き換え
* != nullptrを書き換え
* .
* latency -> control_latency
* .
* Merge branch 'feature/support_engine' into develop
* wip: バッテリー，エンジンの両方に対応
* Merge branch 'develop'
* SHARED -> STATIC
* シミュレーションの開始，終了時に各ウィジェットをリセット
* rosbag record start/stopでspinnerを起動
* move
* setCheckedではシグナル発行を行わない
* 開始 / 停止ボタンをトグルボタンに
* gps -> gnss
* .
* fix
* add rotor viewer
* add: tobas_qwt_wrapper
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* Merge branch 'develop'
* .
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* fix
* .
* wip: tobas_flight_log_gui
* .
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* tobas_std_toolsからtobas_string_toolsを分離
* wip: tobas_flight_log_gui
* .
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* .
* .
* .
* wip: flight log
* wip: flight log
* .
* .
* wip: flight log
* wip: flight log
* .
* wip: flight log
* .
* .
* wip: flight log
* wip: flight log
* wip: flight log
* .
* .
* 使用するディレクトリが存在しなければ作成
* Merge branch 'develop'
* C++20 -> C++23
* アーム時にもrosbag開始可能に
* Merge branch 'develop'
* Merge branch 'feature/separete_logic_and_interface' into develop
* GUIではインターフェーストピックを使用
* フライトログの個数が0ならばその旨を表示
* フィードバックメッセージを記録
* アーム中はログの開始，終了ができないようにした
* rosbag_recorderでTypeAdapterのまま購読
* linux依存を減らした
* Merge remote-tracking branch 'origin/feature/ros1_to_ros2' into develop
* rosbagのパスをローカルとリモートで分ける
* wip: flight_log
* fix: flight_log_recorder
* impl: flight_log
* wip: flight_log
* Contributors: Masayoshi Dohi, dohi, ok10171
