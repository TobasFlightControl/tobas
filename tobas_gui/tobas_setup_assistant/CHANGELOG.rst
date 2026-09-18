^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_setup_assistant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* .
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* .
* Add description
* サブモジュールを削除してベンダーパッケージに
* magic_enum.hpp -> magic_enum/magic_enum.hpp
* magic_enumとcxxoptsのサブモジュールを削除
* 0.0.0 -> 2.16.0
* Merge branch 'feature/consider-fixed-thrust-joint' into jazzy
* .
* frame_type_detectorを別ファイルに分離
* 機体型分類チャートを編集
* .
* .
* NIF -> NIC
* disturbance_observerをcomponent_manager_2に移動
* 動的パラメータサービスにエラーメッセージを追加
* Merge branch 'feature/use-geographiclib' into jazzy
* Add geoid unduration
* .
* thrustジョイントはJSPに含まず自動回転
* コメント改善
* heartbeat_senderをcomponentで実行
* XACRO解析エラーが長い場合にファイル出力
* SAでUADFを読み込んだときに視点をリセット
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* done
* Merge branch 'feature/baro_gnss_fusion' into jazzy-dev
* change: 気圧計をデフォルトで使うように
* fix
* 気圧高度バイアス推定
* 気圧高度の初期値を初期データの平均で決定
* .
* クリック以外のリスト要素選択にも対応
* fix
* fix
* arm/disarm durationを静的パラメータに
* enum struct -> enum class
* C由来の関数を排除
* C由来の関数を排除
* C由来の関数を排除
* Merge branch 'jazzy-dev' into jazzy
* IMUのサンプリング周波数を全体で固定
* RPMフィルタのパラメータを調整可能に
* 外部Poseを受け容れるかどうかをパラメータに
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* .
* .
* ユーザパッケージのlaunchは名前空間内でインクルード
* Show robot name
* QOverload -> qOverload
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/fc2xx_driver' into jazzy-dev
* LPF2のカットオフのデフォルト値をFCの型によって変える
* .
* add: fc2xx.xpp
* modify: New UADFのcolcon buildをバックグラウンドで実行
* wip: colcon build in background
* 400Hz -> 800Hz
* modify: RCLCPP_DEBUG -> qDebug()
* 中心ピッチ角をSAで設定
* fix
* Merge branch 'feature/add_custom_health_check' into jazzy-dev
* add: user-defined health statsu
* fix
* Merge branch 'feature/refactor_rviz_plugin' into jazzy-dev
* fix
  ライセンス条項を追加
  使ってない依存パッケージを削除
  コンポーネントに
  unique_ptrで発行
  トピックを相対パスに
  QoSをUDPに
  制御器側で障害物回避を制御
* Merge pull request `#7 <https://github.com/TobasFlightControl/tobas/issues/7>`_ from TobasFlightControl/feature/objetc_avoidance
  Feature/objetc avoidance
* SRDFへの依存を削除
* delete: robot_state_visualization
* PRコメント反映
* launchファイル修正
* add license
* BSD -> BSD-3-Clause
* add license
* UUID名前空間の中に入れて競合しないように
* .
* Merge branch 'feature/add_namespace' into jazzy-dev
* fix
* delete unecessary namespace specifications
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* add namespace
* QDebugに着色
* .
* プロジェクト内にプロジェクトを作れないように
* Merge branch 'jazzy-dev' into jazzy
* XMLに書き込む値を適当に丸める
* .
* シミュレーションにおけるGNSS位置精度を変更
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* NodeOptoinsをC++側で固定
  不要な処理を行わないように
* delete unecessary include
* Merge branch 'feature/specify_nif' into jazzy-dev
* Remote ConnectionでNIFの指定を強制
* プロジェクトのバージョンが互換性が保証されないものに変わったら動的パラメータを初期化
* プロジェクト名がROSの慣習に沿っているかどうかを確認
* add compile option: -Wswitch-enum
* PosVelコマンドにAccを追加
* 位置制御のゲインを推力に応じてスロットル
* 最低速度を変更
* tobas_constantsをファイル分割
* Merge branch 'feature/update_raspbian_version' into jazzy-dev
* end0 -> eth0
* heading_rate, heading_accelをパラメータに
* Merge branch 'release/v2.12' into feature/refine_img_processing
* .
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* remove unnecessary installation
* .
* .
* Mission Executorの設定画面を追加
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix
* Y軸チルトコプターのレートコマンドを追加
* fix: qDebug
* 不要な定義を削除
* property_treeのセクションを細分化
* Merge branch 'jazzy-dev' into jazzy
* mission executorをcomponentに移動
* Merge branch 'feature/mission_execution' into jazzy-dev
* ミッション実行用のノードを1箇所に統合
* センサの更新確認を削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* wip: yaml double format
* wip: yaml double format
* change: ESC通信断絶の閾値を調整できるようにした
* modify: configurations -> configuration
* network.yamlを自動生成
* wip: network settings
* wip: Network setting page
* .
* .
* fix: 環境変数をトップレベルで設定
* Merge branch 'feature/eskf_external_odometry' into jazzy-dev
* isValid()の内部ではqErrorBoxではなくqWarnBox
* ESKFで各センサを使用するかどうかを選べるようにした
* Merge branch 'feature/impl_rope_plugins' into jazzy-dev
* GUIに吊り下げ荷物の着脱機能を追加
* wip: add to GUI
* .
* gazeboプラグインをSAで追加
* modify: 環境変数の設定方法
* Merge branch 'jazzy-dev' into jazzy
* fix
* fix
* fix: 隠しファイルを表示
* change: T1 -> FC1xx
* change: Follow XDG Base Directory Specification
* .
* .
* fix: UADFを含むパッケージのビルドを試みるのはパッケージがビルド前の場合のみ
* delete: JointRole::kManipulation
* マニピュレーション系は自動では立ち上げない
* modify: KDLツリーのvalidity check
* modify: size() > 0 -> !empty()
* modify: size() == 0 -> empty()
* .
* .
* change: PWM以外のインターフェースも設定可能に (PWMを指定しなくてもエラーを出さない)
* tobas_std -> tbs
* change: バッテリーの内部抵抗を補償した電圧で判定
* modify: 地磁気関連のチェックをデフォルトでオフに
* rename: tobas_matrix_euler_conversion -> tobas_debug_tools
* wip: radio failsafe
* rename: tobas_health_monitor -> tobas_failsafe
* change: SAとGCSでプロジェクトのバージョンチェック
* change: バージョンファイルを保存
* modify: GUIのタイトルにバージョンを表示
* change: 平面マルチのスタビライズコマンドを変更: Accel Yaw -> Angle Throttle
* Merge branch 'feature/sim_world_settings' into jazzy-dev
* remove: simulation page
* .
* .
* change: S.BUSリーダーを立ち上げない場合はFailSafeを避けるためにダミーのRCInputを発行
* change: Pre-Arm CheckにRadio Linkを追加
* modify: S.BUSデバイスのパスをGUIで設定可能に
* change: Flight Management Unit -> Hardware Interface
* change: 追加ジョイントの仕様
  UserActive, UserPassiveタイプを追加
  LandingGear, PassiveWheel, Otherタイプを削除
  JointCommanderでは位置，速度，力をそのまま発行
* .
* .
* .
* modify: aerodynamicsでvalidityチェック
* change: 可変ピッチプロペラの反トルク係数をピッチ角の関数に変更
* fix: Fix column names
* edit data
* edit data
* change: テーブルのヘッダに単位を書く
* modify: DoubleTableの増減ボタンを削除
* .
* fix: launch argumentsを修正
* modify: get_clock()->now() -> now()
* change: Pre-Arm Check, Post-Arm CheckをVehicle Healthにまとめた
* change: 加速度から振動レベルを計算
* .
* /rosoutと外部ファイルにログを出さない
* .
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* modify
* .
* modify: レイアウトを変更
* .
* .
* change: ビルトイン制御器を使わないオプションを追加
* .
* modify
* フレーム型が定義されていなければコントローラの設定を非表示
* 地磁気センサの手法を選択式に
* edit cmakes
* Merge branch 'feature/impl_gazebo_joint_controller' into jazzy-dev
* add: Add JointStateBridgeNode
* .
* .
* 自前のジョイントコントローラを使用
* wip: ジョイントコントローラをros2_controllersからプラグインに変更
* リンクエラーを検出
* fix: Loadで推進系の切り替え
* .
* change: 遠隔接続状態をGCSに表示
* fix: meshesディレクトリは存在しなくてもOK
* add data
* add data
* add data
* add data
* add data
* add data
* add data
* add data
* add data
* add data
* add data
* add data
* .
* rename
* .
* add: Add T-Motor propeller data
* change: Change tmotor propeller data source
* change: ユーザパッケージ
* .
* modify: Set default preset data
* modify: サイズ変更
* modify: blockSignals -> QSignalBlocker
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: propeller data
* modify: warn textを表示
* modify: rcutils_get_env -> ros2::getEnvなど
* modify: Tobasプロジェクトのパスの条件を追加
* modify: GCSの起動時に~/Tobas/colcon_ws/installをAMENT_PREFIX_PATHに加える
* UADFが属するパッケージを自動でビルド
* .
* modify: URDF関連のコードを別パッケージに分離
* .
* modify: パス作成時のエラーハンドリング
* modify: gui::common -> gui::cmn
* modify: 典型的な文字サイズをtobas_gui_commonにまとめた
* modify: 推進系の型を変更する際に警告
* modify: シミュレーションはLOCALHOST_ONLYで立ち上げる
* modify
* modify: Propulsion Systemのレイアウトを改善
* modify: 不要なインストール，エクスポートを削除
* modify: シミュレーションの初期高度を変更
* change: 離着陸アクションを全てのマルチコプターに対応
* fix
* change: マルチのゲインを普通にPIDでも設定できるオプションを追加
* モデル化誤差のデフォルト値を0%に変更
* modify: Add project_paths.xpp
* modify: ミキサーをそれぞれのコントローラパッケージに移動
* delete: pose.xpp
* edit: gazebo.launch.xml.tplxml
* modify: world widget
* fix package name
* 機体形状判定アルゴリズムを編集
* .
* modify
* add: y_axis_tilt_multicopter
* .
* コンパイルオプションを追加
* add: tobas_x_axis_tilt_multi_controller
* .
* rename controllers
* modify
* modify
* Delete OS checks
* change: property_server, ssh_serverの通信範囲をLOCALONLYに
* modify: PropertyServerの名前空間を削除
* fix
* modify: spin_box.xppとscalar_gettter.xppをintとdoubleで分離
* .
* fix
* create ssh endpoint config file
* .
* .
* .
* modify
* add: IPv6
* .
* wip: Remote Connection
* wip: Remote Connection
* modify: Add getPkgShareDir()
* Merge branch 'feature/network_settings' into jazzy-develop
* modify: Remove kTitle
* delete: XXX:
* .
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* .
* modify: 保存時にパスが存在しなければ作る
* wip: mag_calibration
* .
* .
* change: 可変ピッチプロペラの空力特性推定機能を実装
* .
* modify: ウィジェットサイズを最大化
* .
* modify
* change: エンジンダイナミクスの推定機能を実装
* .
* modify
* modify: message
* modify: QCheckBox -> QRadioButton
* modify: 回転翼を持たない場合は設定を無効化
* modify: 追加ジョイントを持たない場合は設定を無効化
* modify: Descriptions of each setting page
* .
* change: プロペラの幾何形状の設定方法を変更
* .
* delete: Manual, BladeTheory
* modify: 2 * M_PI -> M_2PI
* change: プロペラデータをプルダウンで選択できるようにした
* .
* .
* .
* rename: tbs_path -> proj_path
* modify: gazeboをheadlessモードで起動できるように
* modify: デフォルト値をF450に合わせた
* modify: プロジェクト名のデフォルト値に接頭語"tobas\_"をつけた
* Merge branch 'jazzy-develop' into jazzy
* modify: 必要な設定のみ有効化
* change: name(), title()
* change: QTabWidgetではなくQToolBoxを使用．基本設定と追加設定を分ける．
* modify: 並べ替え
* change: UADFロード時にフレーム型を決定
* delete: Delete rotor_axis_t
* format
* modify: enum format
* modify: UrdfParser, KdlParserを定義
* modify: UADFの解析結果をキャプチャ
* modify: xacro解析のエラー出力を表示
* change: JSPにZeroボタンを追加
* add: Robot Properties
* .
* modify: SaveProjectDialogのデフォルトファイル名を設定
* delete: ControllerWidget::description
* modify: aerodynamics設定方法のデフォルト
* modify: QTableWidgetをあまり触れないように
* Merge branch 'feature/define_uadf' into jazzy-develop
* fix: 既存のマーカを削除
* delete: reset()
* edit: thrust stand data
* change: PWMの範囲をdoubleに
* fix: PWMの設定を記載
* .
* fix
* fix: cs, tiltのジョイントコントローラを立ち上げる
* .
* delete: optional_device.xpp
* change: UADFのパスを変更
* fix
* fix
* modify: テーブルの要素ウィジェットを保持しない
* modify: ターゲットの重複チェック
* modify: row label
* .
* modify: Bidirectionalボタンをプッシュボタンに
* fix
* wip
* .
* fix
* fix
* .
* rename: joint_config -> extra_joints
* fix
* .
* 一時的にexportURDFのdeprecated警告を抑制
* fix
* fix
* fix
* fix
* wip
* wip
* wip
* edit xacro parser
* rename: urdf_content -> urdf_text
* modify
* change: 電動モータの最小回転数をGUIで変更できるように
* modify: ユーザ設定読み込みに失敗しても続行
* Merge branch 'feature/fix' into jazzy-develop
* add: dji_9450.csv
* rename: Tobas Package -> Tobas Project
* change: ROS Packageのページを消し，親ディレクトリとプロジェクト名をSaveダイアログで入力
* rename: Tobas Pacakge -> Tobas Project
* TBSプロジェクトをダブルクリックで選択できるように
* modify: FMUがサポートするPWMチャンネル数をチェック
* modify: maintainer name/emailのフォールバック
* change maintainer email
* change: use_gnss, use_baro -> position_source
* change: SAからセンサのタブを消し，センサのパラメータはHardwareWidgetから出す
* fix
* delete: SAのObserverの選択肢を削除
* fix
* add: Add matrix_euler_conversion
* Merge branch 'feature/remove_sensor_covariances' into jazzy-develop
* change: IMUフィルタのパラメータをチューニング画面に追加
* change: tobas_config_serversで動的パラメータを管理
* change: IMU, Mag, Baroの分散を固定
* modify: Delete using namespace std;
* modify: [0], at(0) -> front()
* rename: quat -> quar
* change: adaptive grav noiseはデフォルトではオフ
* change: 重力方向の観測ノイズの分散を適応的に
* change: fix_hoge_noise -> adaptive_hoge_noise
* change: ESKFの静的パラメータを追加
* modify
* modify: Change initial z position: 0.3m -> 1.0m
* change: Hide fixed wing settings
* change: ControlSurfaceの仕様変更
  Gazeboから各舵角を読み取る
  ティルトロータと同様にJointPositionArrayで指令
  channelではなくlink_nameで識別
* change: original.urdfのメッシュパスを相対パスで記述
* change: ヘッダーを削除してNew, Load, Saveボタンに
* change: 各アプリケーション専用のアイコンを作成
* fix: デスクトップアイコンが表示されない問題
* change: URDF BuilderとSetup Assistantを別アプリに
* modify: MSG_FILES -> IFACE_FILES
* fix: msgファイルが存在するときのみ
* change: ユーザメッセージパッケージを自動生成
* add: Add data
* add: dji_inspire1_1345.csv
* modify: Comment out some data
* change: 地磁気のサンプリングレートを変更: 100Hz -> 50Hz
* change: ユーザパッケージのリアルタイムプロセスとインターフェースプロセスを分離
* .
* change: jnt_role_t::LANDING_GEAR
* Merge branch 'jazzy-develop' into feature/mag_drivers
* modify: デフォルトディレクトリ
* add: dji_1045.csv
* csvのコメント業と空行を無視
* .
* RvizでX11を指定
* Merge branch 'jazzy-develop' into jazzy
* delete: EngineConfig/max_speed
* IMU preprocessの動的パラメータを調整可能に
* Merge branch 'feature/engine_model' into jazzy-develop
* ICEプラグインでエンジンの最大回転数を考慮
* エンジンモデルを修正
* .
* fix: Fix wild cards: [^.]* -> *
* cmake version: 3.10 -> 3.25
* CONFIGURE_DEPENDS
* messageのモードをSTATUSに
* fix: ユーザパッケージが存在すれば何も新規作成しない
* sort includes
* change: include order & group
* cmake-format
* Fix typos
* clang-format
* clang-format
* clang-format
* clang-format
* clang-format
* real.launchにマルチプロセスオプションを追加
* use_sim_timeのデフォルト値をfalseに
* ComponentManagerのスレッドセーフ化に伴いTimerEventを削除
* タイマー起動に戻した
* fix
* ノードの起動順序を指定
* 待機時間を延長
* .
* 起動順入れ替え
* robot_state_publisherをgazebo, realそれぞれで立ち上げる
* エンジンの最大回転数の設定
* パラメータチューニングにRC Teleopを追加
* fix: num_sbus_channles -> num_sbus_channels
* .
* SAでS.BUSチャネル数を取得
* LookAtPositionプラグインを埋め込む
* basic.world -> empty.world
* /tf, /tf_staticをドローンの名前空間に入れる
* delete: xml version
* delete: xml version
* Merge branch 'main' into develop
* 依存解決
* .
* fix: package_generator.cpp
* .
* .
* MoveItへの依存を回避
* edit: TabWidget
* .
* propulsion typeが変わったら設定をリセット
* PropulsionSystemのtypeをcheckboxで選択
* .
* updateInternalDataStructuresをSetupAssistantWidgetで行う
* fix
* メッシュパス解決
* オリジナルURDFのメッシュファイルも変更
* add: マーカ発行機能をrotor_marker_publisherに移植
* delete: blockSignals
* fix
* fix
* .
* fix
* fix
* Author Informationのデフォルトを入れる
* fix
* .
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* .
* delete: setFocusPolicy
* delete: General/Channel
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* .
* DoubleRange, IntRangeの仕様変更
* .
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* wip: Engine GUI Settings
* .
* fix
* 共通のシグナルクラスを作成
* common.hpp -> constants.hpp
* qobject_cast -> qt::qPointerCast
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* .
* dekete: no_select
* Propulsion Systemのページを再編成
* .
* kdl_parserをtobas_kdl_parserに分離
* Aso -> T1
* rename: aso -> t1
* change default magnetometer update rate: 20Hz -> 100Hz
* impl: ICEPropulsionSystemHandlerNode
* jnt_hw_iface_t -> hw_iface_t
* PWMと値の範囲をRangeで保持
* PwmConfigがジョイントに依存しないように
* Merge branch 'feature/delete_euler_topic' into develop
* "euler"トピックを削除
* 外乱補償デフォルトオフ
* delete: Time Reference Server
* edit: join.hpp
* == nullptrを書き換え
* != nullptrを書き換え
* .
* impl: tobas_landing_detection
* ノードの接続j状態をPreArmCheckで確認
* アクティブティルトの制御器を外乱補償に対応
* 並進の外乱補償はデフォルトでオン
* tobas_msgs::RCInputをTypeAdapterに対応
* fix
* RC操縦モードを追加
* wip: コマンド変更
* fix
* ノイズのデフォルト値を変更
* .
* IMU, Magの初期バイアスの大きさを固定
* accel yaw コマンドを追加
* Latency PublisherをDShot Driverに移植
* fix
* RC入力関連のノードをSim, Realの両方で立ち上げる
* 外乱補償をデフォルトではオフに
* Merge branch 'feature/support_engine' into develop
* .
* .
* turning_directionの取得をメソッド化
* fix
* wip: バッテリー，エンジンの両方に対応
* rename: rotor_plugin -> electric_propulsion_system_plugin
* メッシュファイルの存在を確認
* rosrun, roslaunchの前にsource
* fix
* change default propeller diameter: 10 inch -> 9 inch
* Merge branch 'develop'
* SHARED -> STATIC
* rviz依存のコードをtobas_rviz_wrapperに移動
* DroneのYAMLエンコードを文字列に
* fix
* .
* SAWにPreArmCheckのページを追加
* PreArmCheckerとPostArmCheckerを分離
* シミュレーションの開始，終了時に各ウィジェットをリセット
* .
* URDF Builderを移動など
* Programモードを廃止してLoiterモードを追加
* change namespaces
* ボタンサイズを変更
* add: base_pose_commander
* .
* コマンドメッセージを改名
* gps -> gnss
* Turning DirectionをGeneralに移動
* wip: tobas_flight_log_gui
* Merge branch 'develop'
* tobas_std_toolsからtobas_string_toolsを分離
* wip: flight log
* .
* wip: flight log
* .
* ros2::TimerPtr -> QTimer
* .
* move
* Joint ConfigurationでPWM関連の設定ができるように
* PwmConfigを定義
* PreArmCheck + PostArmCheck
* rotor_checkerを移動
* .
* .
* 地磁気バイアス推定の項目を追加
* 地磁気センサのバイアス (Hard Iron) をローカルで加える
* fix magnetometer default parameters
* fix
* .
* .
* .
* .
* "Reverse"フィールドを追加
* fix
* edit: joint_command_handler.cpp
* JointCommandHandler -> JointsHandler
* sensor_msgs/JointState -> tobas_msgs/JointStateArray
* add: joint_state_bridge
* .
* 外乱補償の設定
* namespace propulsion_system -> propulsion
* fix
* load時にblockSignal
* wip: JointConfigurationWidget
* fix
* add: ActiveTiltMultirotorPID setting
* .
* .
* .
* edit: controller.cpp
* Merge branch 'feature/integrate_joint_configuration' into develop
* finish
* std::unordered_set -> QSet
* onOpenedでJoint Configを更新
* .
* .
* .
* wip
* wip
* wip
* ジョイントリミットをkdl::Treeから取得
* add: JointCommandHandlerNode
* joint_interface -> command_interface
* Merge branch 'feature/aso_ver1_to_ver2' into develop
* adc_driver -> battery_driver
* state_estimation_eskf -> tobas_eskf
* Merge branch 'develop'
* updateInternalDataStructuresの返り値を変更など
* move
* kdlのファイル名を変更
* .
* .
* fix
* JointConfig.role, RotorConfig.tilt_joint_nameを追加
* C++20 -> C++23
* ジョイントコントローラごとにconfigファイルを作成
* .
* カスタム制御機と観測機でパッケージ名とプラグイン名を設定させる
* actionsPackageを自動判定
* .
* GazeboではComponent Containersを別プロセスで立ち上げる
* fix
* 起動を遅延
* topic_throttle_nodeを1つにまとめた
* fix
* fix
* fix
* GPSの周波数を上げた
* move: MatrixEulerConverter
* センサのフィルタリングと分散計測をGazebo, Realからtobas_preprocessに移行
* change update rate
* ロータの周波数ノイズを実装
* プロペラジョイントのlimitを自動削除
* fix: jointpos_commander
* fix: joint_control.yaml
* fix: selected_links.xpp
* .
* ControlSurfaceをstd::mapで管理
* RotorConfigをstd::mapで管理
* common_realtime.launchのstandaloneバージョンを用意
* wip: モータ異常検知
* rotor_indexとchannelを混同している部分を修正
* battery_lpfを削除
* シミュレーション時にコアソフトをPC側で実行
* HALトピックをスロットル
* fix: hitl.launch.py.tplpy
* fix
* impl: HITL
* wip: simulation
* ユーザノードをinterface.launchに移動
* fix: real_realtime.launch.py.tplpy
* .
* バックアップファイルをROSパッケージの外に出した
* .
* "Random"ボタンを追加
* ロータの状態をRotorSpeedArrayではなくRotorStateArrayで管理
* keyboard_teleopを生成しない．
* fix
* fix
* .
* fix
* fix
* fix
* .
* ユーザ用Pythonパッケージを作成
* delete: ElectroDynamics
* モータのシミュレーションをダイナミクスベースに
* モータの回転数制御
* Inertia -> Inertial
* Merge branch 'develop'
* change QoS of DRS
* PositionYawを削除
* GPSの遅延を0.2secから0.1secに変更
* リアルタイムプロセスとインターフェースプロセスのlaunchファイルを分離
* Merge branch 'feature/separete_logic_and_interface' into develop
* impl: tobas_dparam
* アクションサーバではthrottled/odomを購読
* wip: tobas_ros_interface
* wip: tobas_ros_interface
* gazeboのIMU LPFを削除
* コンポーネントのロードに失敗したらシャットダウン
* ComponentManagerにロードするときの待ち時間を増やす
* add: gazebo cpu handler
* onInitを削除
* onInitを削除
* 優先度を4段階から3段階に変更
* linux依存を減らした
* Merge remote-tracking branch 'origin/feature/ros1_to_ros2' into develop
* wip: rosbag_recorder
* fix
* delete: latency_throttle
* fix
* .
* ノードの起動を遅延させる
* 複数のComponentManagerを一つのプロセスで動作させる
* fix
* .
* wip
* real.launch.xml -> real.launch.py
* size
* fix
* fix
* wip: tobas_gui
* wip: tobas_gui
* wip: tobas_gui
* Inertiaを可視化
* const Drone&を保持
* delete: tobas_calibration
* wip: tobas_hardware_setup
* add icon
* delete icon
* add: tobas_gui_common (std::formatを使うため)
* .
* fix
* センサ処理ノードのパラメータをptreeではなくrosparamで管理
* property_toolsを分割
* rename: simple\_(service, action, param)_client -> sync\_(service, action, param)_client
* joint_command_handlerのパラメータを設定
* 同期クライアントを極力使わないように
* .
* outputを指定
* .
* wip
* fix
* ログレベルを設定
* fix
* .
* wip
* fix
* wip
* fix
* メッシュファイルへのパスを修正
* mesh -> meshes
* wip
* fix
* wip
* fix
* wip
* .
* fix
* fix
* パッケージビルドを別スレッドに
* add: wait_spinner
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* .
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* add submodules
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* move
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* .
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip
* wip
* .
* impl: joint_state_publisher
* add: frame_tree
* wip: tobas_setup_assistant
* impl: tobas_setup_assistant/xml_nodes
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* wip: tobas_qt_tools
* .
* .
* wip: tobas_setup_assistant
* Contributors: Masayoshi Dohi, dohi, ok10171
