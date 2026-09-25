^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_qt_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Hide the title bar buttons.
* 色変更
* Merge branch 'feature/gcs_layout' into jazzy
* delete unecessary "this->"
* style: use explicit decimal literals
* HPositionBarWidget -> ProgressBar
* set margin zero
* .
* よく使う色を定義
* .
* delete style sheet
* make group
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* Merge branch 'feature/scp-progress' into jazzy-dev
* テキストが空なら何も表示しない
* Merge branch 'jazzy-dev' into jazzy
* .
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* 表示上のコマンド番号を1-indexedに
* リストに行番号を表示
* QOverload -> qOverload
* Merge branch 'feature/fix_ip_address' into jazzy-dev
* IPを自動修正
* .
* .
* ログ名の履歴を残す
* Merge branch 'jazzy-dev' into jazzy
* modify: thread.hpp
* launchのプロセスをQProcessで管理
* Merge branch 'feature/add_custom_health_check' into jazzy-dev
* add: user-defined health statsu
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* QDebugに着色
* Merge branch 'jazzy-dev' into jazzy
* add compile option: -Wswitch-enum
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* スクロールエリアの背景を透明に
* .
* disableでもテキストは通常表示
* .
* chronoに対応
* 完全にラズパイがシャットダウンされるまで適当に待つ
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: qDebug
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* ビルドエラーメッセージが長ければファイル出力
* .
* 時刻ではなくスピナーに変更
* modify 動作するインジケータを表示
* modify: シミュレーションの開始処理を別スレッドで実行
* fix: MetaTypeの定義を保証
* 別スレッドの実行を簡略化
* Merge branch 'feature/impl_rope_plugins' into jazzy-dev
* GUIに吊り下げ荷物の着脱機能を追加
* wip: add to GUI
* Merge branch 'jazzy-dev' into jazzy
* modify: size() > 0 -> !empty()
* modify: size() == 0 -> empty()
* modify: PWM以外のジョイントもテスト可能に
* tobas_std -> tbs
* modify: S.BUSデバイスのパスをGUIで設定可能に
* change: sliderReleased -> valueChanged
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* fix
* edit cmakes
* modify: グリッドレイアウトを初期化
* modify
* modify: Qtイベント処理で画面更新を安定化
* fix: トピック張替え後にQtイベントを全て処理
* modify: blockSignals -> QSignalBlocker
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* fix: paintEvent
* .
* modify: Tobasプロジェクトのパスの条件を追加
* wip: キャリブレーション済みかどうか表示
* .
* modify: Base Pose Commanderをdegree指定に
* .
* コンパイルオプションを追加
* Delete OS checks
* modify: spin_box.xppとscalar_gettter.xppをintとdoubleで分離
* add: IPv6
* .
* wip: Remote Connection
* Merge branch 'feature/network_settings' into jazzy-develop
* modify: 無効な文字を拒否
* modify: 接続中にアプリを落とそうとしたら警告
* modify: Reset PasswordEdit
* modify: font.cpp
* add: hostname setting widget
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* wip: パスワードを隠す
* wip
* wip: mag_calibration
* wip: mag_calibration
* change: qt::CombBox::sort()
* Merge branch 'jazzy-develop' into jazzy
* modify: add method
* change: QTabWidgetではなくQToolBoxを使用．基本設定と追加設定を分ける．
* modify
* modify: QTableWidgetをあまり触れないように
* Merge branch 'feature/define_uadf' into jazzy-develop
* .
* fix
* wip
* wip
* change: ROS Packageのページを消し，親ディレクトリとプロジェクト名をSaveダイアログで入力
* change maintainer email
* modify: Delete using namespace std;
* modify: [0], at(0) -> front()
* Merge branch 'jazzy-develop' into feature/mag_drivers
* .
* Merge branch 'jazzy-develop' into jazzy
* .
* .
* fix: Fix wild cards: [^.]* -> *
* cmake version: 3.10 -> 3.25
* CONFIGURE_DEPENDS
* messageのモードをSTATUSに
* refactor: インクルードパスを編集
* sort includes
* change: include order & group
* cmake-format
* clang-format
* clang-format
* clang-format
* edit: position_bar_widget.xpp
* PositionBarWidgetのテキストを綺麗に配置
* add method
* wip: RC Calibration
* ワールド名を追加
* delete: xml version
* edit: TabWidget
* .
* .
* .
* qobject_cast -> qt::qPointerCast
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* RCビューアのポイントサイズを揃えた
* Merge branch 'develop'
* SHARED -> STATIC
* rviz依存のコードをtobas_rviz_wrapperに移動
* .
* URDF Builderを移動など
* edit: toggle_switch.xpp
* edit: main_widget.cpp
* wip: simulation
* setCheckedではシグナル発行を行わない
* トグルボタンを実装
* add: base_pose_commander
* .
* add: tobas_qwt_wrapper
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* Merge branch 'develop'
* wip: tobas_flight_log_gui
* wip: tobas_flight_log_gui
* .
* wip: tobas_flight_log_gui
* fix
* wip: flight log
* .
* Merge branch 'feature/integrate_joint_configuration' into develop
* edit: combo_box.xpp
* Merge branch 'develop'
* C++20 -> C++23
* "Random"ボタンを追加
* モータの速度調整機能を実装
* Merge branch 'develop'
* Merge remote-tracking branch 'origin/feature/ros1_to_ros2' into develop
* impl: flight_log
* fix: progress_dialog
* fix: rcin_calibration
* fix
* 再描画は各コンポーネント内でやる
* wip: control system
* wip: control system
* wip: control system
* wip: control system
* wip: control system
* fix
* fix
* fix: rviz.xpp
* fix
* wip: tobas_gui
* add: parameter_tuning_gui
* wip: tobas_hardware_setup
* delete icon
* ads1220
* property_toolsを分割
* .
* fix
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
* シグナルスロットの接続をモダンに
* impl: joint_state_publisher
* wip: tobas_setup_assistant
* wip: tobas_setup_assistant
* impl: tobas_qt_tools
* wip: tobas_qt_tools
* wip: tobas_qt_tools
* wip: tobas_qt_tools
* Contributors: Masayoshi Dohi, dohi, ok10171, pi
