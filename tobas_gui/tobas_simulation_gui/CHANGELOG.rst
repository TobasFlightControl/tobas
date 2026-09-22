^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_simulation_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'feature/log_plot_scale' into jazzy
* .
* コメント改善
* サービスコールの制限時間を無制限に
* 予期せぬ通信断絶時にダイアログを表示
* .
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/baro_gnss_fusion' into jazzy-dev
* modify
* Merge branch 'jazzy-dev' into jazzy
* .
* Merge branch 'jazzy-dev' into jazzy
* .
* シミュレーションクラッシュ時の出力を保存
* シミュレーションがこけたときにキャッチして終了
* .
* .
* wip: launchプロセスを監視
* launchのプロセスをQProcessで管理
* add license
* add license
* ドローンの名前空間を絶対パスで指定
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* delete unecessary include
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* .
* add compile option: -Wswitch-enum
* .
* 加速度指令を追加
* PosVelコマンドにAccを追加
* fix
* デフォルトのワールドをbaylandsに
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
* change: CommandLevel -> Priority
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: qDebug
* 不要な定義を削除
* property_treeのセクションを細分化
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/mission_execution' into jazzy-dev
* .
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* ビルドエラーメッセージが長ければファイル出力
* modify: configurations -> configuration
* modify: シミュレーションの開始処理を別スレッドで実行
* modify: Gazeboのlaunch, killをスレッド化
* 別スレッドの実行を簡略化
* SshClientのQtラッパーを実装して使用
* modify: SSHClient -> SshClient
* Merge branch 'feature/impl_rope_plugins' into jazzy-dev
* GUIに吊り下げ荷物の着脱機能を追加
* wip: add to GUI
* Merge branch 'jazzy-dev' into jazzy
* modify: size() > 0 -> !empty()
* tobas_std -> tbs
* modify: 手動操縦が有効ならGUI Teleoperationは無効
* modify: 外部からディスアームされたら強制終了
* fix
* modify: 変化した選択肢だけ更新
* modify: S.BUSデバイスのパスをGUIで設定可能に
* rename: joint_position_commander -> joint_commander
* change: 追加ジョイントの仕様
  UserActive, UserPassiveタイプを追加
  LandingGear, PassiveWheel, Otherタイプを削除
  JointCommanderでは位置，速度，力をそのまま発行
* .
* .
* modify
* change: Gazeboレンダリングの開始を待つ
* wip: RenderInfo
* modify
* fix: kWaitForServiceを5分に
* gazebo起動時のサービスの待ち時間を最大に
* デフォルトのワールドをシンプルなものに変更
* change: 全てのワールドをRealFlight視点に
* change: デフォルトのworldをflight_trainingに変更
* .
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: プロジェクトのビルドを別スレッドで行う
* modify: kill_gazeboを1つの関数に
* edit cmakes
* modify: シミュレーションの起動に失敗したらシャットダウン
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: rcutils_get_env -> ros2::getEnvなど
* modify: GCSの起動時に~/Tobas/colcon_ws/installをAMENT_PREFIX_PATHに加える
* UADFが属するパッケージを自動でビルド
* modify: gui::common -> gui::cmn
* modify: 典型的な文字サイズをtobas_gui_commonにまとめた
* .
* .
* modify: シミュレーション起動中はプロジェクトを読み込めないようにした
* wip: Wait gazebo to shut down
* modify: シミュレーションの初期高度を変更
* modify: Add project_paths.xpp
* fix
* modify: Base Pose Commanderをdegree指定に
* modify
* .
* modify: シミュレーションの初期位置を指定できるように
* modify: world widget
* add user debug mode
* modify: QCheckBox -> QRadioButton
* .
* コンパイルオプションを追加
* add pos_vel_pitch_yaw command
* Delete OS checks
* modify: PropertyServerの名前空間を削除
* Merge branch 'feature/network_settings' into jazzy-develop
* delete: XXX:
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* .
* rename: tbs_path -> proj_path
* modify
* Merge branch 'jazzy-develop' into jazzy
* modify: enum format
* modify: UrdfParser, KdlParserを定義
* modify: UADFの解析結果をキャプチャ
* rename: errer.hpp -> error.hpp
* Merge branch 'feature/define_uadf' into jazzy-develop
* change: UADFのパスを変更
* fix
* wip
* Merge branch 'feature/fix' into jazzy-develop
* HITLをコメントアウト
* rename: Tobas Package -> Tobas Project
* change: ROS Packageのページを消し，親ディレクトリとプロジェクト名をSaveダイアログで入力
* rename: Tobas Pacakge -> Tobas Project
* change maintainer email
* modify: Delete using namespace std;
* modify: [0], at(0) -> front()
* fix: scpPutの前に親ディレクトリを作成
* modify: switch case文を追加
* Merge branch 'jazzy-develop' into feature/mag_drivers
* GUIからSubscriberPtrを排除
* Merge branch 'jazzy-develop' into jazzy
* .
* fix: Fix wild cards: [^.]* -> *
* cmake version: 3.10 -> 3.25
* CONFIGURE_DEPENDS
* messageのモードをSTATUSに
* sort includes
* change: include order & group
* cmake-format
* clang-format
* clang-format
* ワールド名を追加
* basic.world -> empty.world
* fix: joint_position_commander.cpp
* delete: xml version
* Merge branch 'main' into develop
* 依存解決
* .
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* kdl_parserをtobas_kdl_parserに分離
* smart_ptr = nullptr -> smart_ptr.reset
* == nullptrを書き換え
* != nullptrを書き換え
* wip: コマンド変更
* Merge branch 'feature/support_engine' into develop
* armingの処理をrotor controllerでやる
* wip: バッテリー，エンジンの両方に対応
* rosrun, roslaunchの前にsource
* Merge branch 'develop'
* SHARED -> STATIC
* edit: simulation.cpp
* シミュレーションの開始，終了時に各ウィジェットをリセット
* シミュレーション起動時にresetTime
* closeEventでシミュレーションの子プロセスを破棄
* impl: Joint Position Commander
* fix
* wip: simulation
* setCheckedではシグナル発行を行わない
* 停止時にwind parameterをリセット
* スクロール可能に
* wip: joint_position_commander
* ボタンサイズを変更
* move
* rename: static configuration -> simulation settings
* edit world settings
* トグルボタンを実装
* add: base_pose_commander
* Merge branch 'develop'
* .
* Armingを専用メッセージに
* シミュレーション終了時にarming_をリセット
* Merge branch 'develop'
* C++20 -> C++23
* .
* .
* シミュレーション時にコアソフトをPC側で実行
* fix: simulation.cpp
* fix
* impl: HITL
* wip: simulation
* fix
* .
* .
* ros1 -> ros2: simulation
* Contributors: Masayoshi Dohi, dohi, ok10171
