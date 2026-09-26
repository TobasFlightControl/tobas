^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_control_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.5 (2026-09-26)
-------------------
* Add dependencies
* Contributors: Masayoshi Dohi

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
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* fix
* Add description
* magic_enum.hpp -> magic_enum/magic_enum.hpp
* magic_enumとcxxoptsのサブモジュールを削除
* 0.0.0 -> 2.16.0
* Merge branch 'feature/use-geographiclib' into jazzy
* altitude -> height_msl, ellipsoid_height -> height_wgs84
* geographic関連をクラス化
* GNSS関連の計算をgeographilibに
* .
* モード1仕様に変更
* modify
* 色変更
* Merge branch 'feature/gcs_layout' into jazzy
* style: use explicit decimal literals
* .
* HPositionBarWidget -> ProgressBar
* fix
* .
* よく使う色を定義
* edit RC input viewer
* status_viewer -> health_viewer
* .
* アーム状態をバナーで表示
* .
* ステータスを文字で表示
* make group
* Merge branch 'feature/english_translation' into jazzy
* English translation
* modify: catmull_rom
* Merge branch 'feature/spline_waypoint' into jazzy
* .
* CatmullRomPathを共通化
* スプラインを折れ線としてマップ上に表示
* スプライン中間だったらtolerance系を無効化
* ミッションプランナーにStop At Waypointフィールドを追加
* pause -> stop_at_waypoint
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* GNSS衛生数をGCSに表示
* 高度をテキストで表示
* .
* .
* wip: show altitude
* modify: プロポのリンクが切れたらRCビューアをリセット
* Merge branch 'feature/scp-progress' into jazzy-dev
* .
* クリック以外のリスト要素選択にも対応
* C由来の関数を排除
* C由来の関数を排除
* Add mission example
* Merge branch 'jazzy-dev' into jazzy
* .
* modify: 前置インクリメントを使用
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* .
* 新しいウェイポイントは最後の点の少し東に配置
* modify: ダブルクリックでコマンドを確定
* .
* 海抜高度で指定可能に
* 表示上のコマンド番号を1-indexedに
* change layout
* リストに行番号を表示
* 最後に実行中だったミッション番号を返す
* ミッションの実行可能性をチェック
* ミッションの読み込み機能を実装
* .
* NaNならばデフォルト値を使う
* fix
* ミッション保存機能を実装
* .
* tobas_mission_msgs/action/ExecuteMissionの型変更に対応
* QOverload -> qOverload
* .
* Merge branch 'jazzy-dev' into jazzy
* modify: RCLCPP_DEBUG -> qDebug()
* Merge branch 'feature/add_custom_health_check' into jazzy-dev
* add: user-defined health statsu
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
* move
* Merge branch 'feature/smooth_command_switching' into jazzy-dev
* お度目取りメッセージをOdometryからOdometryWithCovarianceStampedに変更
* 行番号を非表示に
* テーブルのスクロール位置を自動で調整
* こんそーる文字にカーソルを合わせると全文が表示されるように
* add compile option: -Wswitch-enum
* .
* .
* MapItemViewにオーバーラップ時の優先度を設定
* ドラッグアンドドロップしやすく
* オーバーズーム時にもオブジェクトをうまくスケール
* set arrow anchor point
* change gps arrow image
* .
* ズームレベルの最小値を変更
* 最低速度を変更
* tobas_constantsをファイル分割
* ミッションの高度はアーム地点を基準にする
* heading_rate, heading_accelをパラメータに
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into release/v2.12
* fix
* Merge branch 'jazzy-dev' into jazzy
* .
* タイルサーバをGoogleに変更
* .
* .
* qmlformat
* オーバーズームに対応
* スクロールエリアの背景を透明に
* レイアウト変更
* .
* add: tobas_tile_proxy
* change: ミッションに優先度を定義
* ミッションにエラーコードを追加
* .
* .
* disableでもテキストは通常表示
* change: RTLでLandまでやる
* .
* .
* ミッションコマンドフィールドでデフォルト値使うかオーバーライドするかを選択可能に
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Altitude Toleranceのデフォルト値を変更: 0.8m -> 2.0m
* fix: qDebug
* 不要な定義を削除
* Merge branch 'jazzy-dev' into jazzy
* .
* .
* .
* Merge branch 'feature/mission_execution' into jazzy-dev
* add auto_heading option
* アクションクライアントを対応
* .
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* modify: fabs -> std::abs
* Merge branch 'jazzy-dev' into jazzy
* tobas_std -> tbs
* modify: 手動操縦が有効だったらミッションを実行できない
* fix
* change: ウェイポイントのAcceptance Radiusを水平と垂直で別々に設定
* change: アクションで最大速度などをもとに最速軌道を生成
* .
* change: 世界座標系をNWU座標系からENU座標系に変更
* change: Pre-Arm CheckにRadio Linkを追加
* .
* .
* .
* .
* Fix: マッププロバイダを指定
* .
* .
* fix: Pose Viewer
* fix
* change: VehicleHealthの内容を肯定的に
* GNSSの状態を表示
* .
* change: Pre-Arm Check, Post-Arm CheckをVehicle Healthにまとめた
* modify: レイアウト変更
* change: 加速度から振動レベルを計算
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* modify
* edit cmakes
* modify
* .
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* .
* コンパイルオプションを追加
* メッセージを移動など
* Delete OS checks
* modify: spin_box.xppとscalar_gettter.xppをintとdoubleで分離
* modify: Change namespace: gcs -> ctrl
* modify: Add getPkgShareDir()
* Merge branch 'feature/network_settings' into jazzy-develop
* delete: XXX:
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* wip: mag_calibration
* change: 可変ピッチプロペラの空力特性推定機能を実装
* .
* Merge branch 'jazzy-develop' into jazzy
* modify: enum format
* modify: QTableWidgetをあまり触れないように
* change maintainer email
* modify: Delete using namespace std;
* fix
* modify: 0%未満のバッテリー残量を0に
* change: 各アプリケーション専用のアイコンを作成
* Merge branch 'jazzy-develop' into feature/mag_drivers
* .
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* .
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* .
* .
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* wip: Separate ROS and Qt threads
* .
* wip: Separate ROS and Qt threads
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
* clang-format
* .
* .
* CPU温度の閾値変更
* .
* PositionBarWidgetのテキストを綺麗に配置
* delete: xml version
* qobject_cast -> qt::qPointerCast
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* fix
* Merge branch 'feature/delete_euler_topic' into develop
* "euler"トピックを削除
* smart_ptr = nullptr -> smart_ptr.reset
* == nullptrを書き換え
* != nullptrを書き換え
* .
* デフォルト値を変更
* tobas_std_msgs/Message: stamp -> header
* ノードの接続j状態をPreArmCheckで確認
* RCビューアのポイントサイズを揃えた
* General Purpose Switchを削除
* RCスイッチ増設 + キルスイッチ実装
* tobas_msgs::RCInputをTypeAdapterに対応
* wip: コマンド変更
* Merge branch 'feature/support_engine' into develop
* use boost::polymorphic_pointer_downcast
* BatteryViewerとEngineViewerをstackで切り替える．
* wip: バッテリー，エンジンの両方に対応
* Merge branch 'develop'
* .
* fix
* .
* SHARED -> STATIC
* .
* OrientationをAttitudeとHeadingに分離
* PreArmCheckの各項目をPASSED, FAILED, IGNOREDで表現
* シミュレーションの開始，終了時に各ウィジェットをリセット
* Enable/Disableで色を変える
* Programモードを廃止してLoiterモードを追加
* change namespaces
* コマンド系のメッセージをtobas_command_msgsに移動
* gps -> gnss
* ロータの生存状況をトピックで発行
* Merge branch 'develop'
* .
* .
* fix
* fix
* .
* Armingを専用メッセージに
* 地磁気オフセットをチェック
* ジャイロノイズをPostArmCheckに追加
* fix
* edit: status_viewer
* .
* FIX_3Dでないと位置を表示しない
* バッテリー電流を表示
* fix
* Merge branch 'develop'
* .
* C++20 -> C++23
* TitleCased.hpp -> snake_cased.hpp
* TitleCased.hpp -> snake_cased.hpp
* PreArmCheckにESCとの通信状態を含めた
* QMLの不要なフィールドを削除
* .
* モータが壊れたらメータの背景色を赤に
* RotorConfigをstd::mapで管理
* rotor_indexとchannelを混同している部分を修正
* ロータの状態をRotorSpeedArrayではなくRotorStateArrayで管理
* モータの速度調整機能を実装
* .
* モータの回転数制御
* OKをデフォルトに
* Merge branch 'develop'
* Merge branch 'feature/separete_logic_and_interface' into develop
* GUIではインターフェーストピックを使用
* wip: tobas_ros_interface
* .
* バッテリー，CPUのバーを値によって色分け
* PreArmCheckにCPU温度を含めた
* PreArmCheckにCPU温度を含めた
* コンソールメッセージの数を減らした
* ConsoleWidgetをControlSystemWidgetに埋め込んだ
* linux依存を減らした
* Merge remote-tracking branch 'origin/feature/ros1_to_ros2' into develop
* impl: rosbag_recorder
* fix: throttles_viewer.cpp
* fix
* .
* scaling
* scaling
* 再描画は各コンポーネント内でやる
* fix
* wip: control system
* wip: control system
* wip: control system
* wayPoint -> waypoint
* .
* .
* wip: control system
* wip: control system
* wip: control system
* wip: control system
* wip: control system
* wip: control system
* wip: control system
* .
* wip: control system
* wip: control system
* size
* wip: control system
* rename
* tobas_mission_planner -> tobas_control_system
* Contributors: Masayoshi Dohi, dohi, ok10171
