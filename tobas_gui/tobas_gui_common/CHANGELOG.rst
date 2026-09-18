^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_gui_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.16.1 (2026-09-18)
-------------------

2.16.0 (2026-07-29)
-------------------
* .
* Merge branch 'feature/prepare-for-apt-release' into jazzy
* .
* hardcode some paths
* Add description
* 0.0.0 -> 2.16.0
* .
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/save_rpm_control_gain_in_project_folder' into jazzy-dev
* done
* fix: scp_get, scp_putのファイルサイズの型を64ビット整数に
* Merge branch 'feature/scp-progress' into jazzy-dev
* ScpGetと同様にScpPutをアクションに
* アクションフィードバックのコールバックを設定可能に
* Merge branch 'jazzy-dev' into jazzy
* Merge branch 'feature/impl_mission_planner' into jazzy-dev
* ミッション保存機能を実装
* Merge branch 'jazzy-dev' into jazzy
* modify: New UADFのcolcon buildをバックグラウンドで実行
* wip: colcon build in background
* launchのプロセスをQProcessで管理
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* .
* fix
* delete unecessary namespace specifications
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* fix: ローカルビルド時は毎回キャッシュをクリア
* リモートビルドで再チャレンジはしない
* .
* .
* Merge branch 'jazzy-dev' into jazzy
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* fix cmake args
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: qDebug
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* ビルドエラーメッセージが長ければファイル出力
* network.yamlを自動生成
* fix: MetaTypeの定義を保証
* QThreadをcppに記述
* 別スレッドの実行を簡略化
* SshClientのQtラッパーを実装して使用
* modify: SSHClient -> SshClient
* Merge branch 'jazzy-dev' into jazzy
* fix
* fix
* fix
* fix: 隠しファイルを表示
* Merge branch 'feature/version' into jazzy-dev
* change: 書き込み時にFCのバージョンチェック
* add: tobas_version
* tobas_std -> tbs
* change: SAとGCSでプロジェクトのバージョンチェック
* .
* change: バージョンファイルを保存
* modify: GUIのタイトルにバージョンを表示
* fix
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* modify
* 依存解決
* fix: プロジェクトのビルドを別スレッドで行う
* edit cmakes
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: GCSの起動時に~/Tobas/colcon_ws/installをAMENT_PREFIX_PATHに加える
* UADFが属するパッケージを自動でビルド
* modify: gui::common -> gui::cmn
* modify: 典型的な文字サイズをtobas_gui_commonにまとめた
* modify: Add project_paths.xpp
* .
* コンパイルオプションを追加
* Delete OS checks
* wip: Remote Connection
* Merge branch 'feature/network_settings' into jazzy-develop
* delete: XXX:
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* rename: tbs_path -> proj_path
* modify
* Merge branch 'jazzy-develop' into jazzy
* modify: enum format
* fix: add dependency on tobas_qt_tools
* Merge branch 'feature/define_uadf' into jazzy-develop
* change: UADFのパスを変更
* wip
* rename: Tobas Package -> Tobas Project
* change: ROS Packageのページを消し，親ディレクトリとプロジェクト名をSaveダイアログで入力
* rename: Tobas Pacakge -> Tobas Project
* TBSプロジェクトをダブルクリックで選択できるように
* .
* change maintainer email
* Merge branch 'feature/remove_sensor_covariances' into jazzy-develop
* change: IMUフィルタのパラメータをチューニング画面に追加
* change: IMU, Mag, Baroの分散を固定
* modify: Delete using namespace std;
* change: 各アプリケーション専用のアイコンを作成
* change: URDF BuilderとSetup Assistantを別アプリに
* change: ユーザメッセージパッケージを自動生成
* Merge branch 'jazzy-develop' into feature/mag_drivers
* RvizでX11を指定
* .
* Merge branch 'jazzy-develop' into jazzy
* IMU preprocessの動的パラメータを調整可能に
* fix: setup.bash -> local_setup.bash
* .
* fix: Fix wild cards: [^.]* -> *
* cmake version: 3.10 -> 3.25
* CONFIGURE_DEPENDS
* ビルドに失敗したらクリーンビルド
* messageのモードをSTATUSに
* refactor: インクルードパスを編集
* sort includes
* change: include order & group
* cmake-lint
* cmake-format
* .
* rename: targets_to_install -> TARGETS_TO_INSTALL
* clang-format
* clang-format
* fix: --symlink-installを削除
* gccのパスを指定
* パラメータチューニングにRC Teleopを追加
* delete: xml version
* Merge commit 'd9019f46cc95d0aa84d4c1ecd6220edb6b4c7357'
* == nullptrを書き換え
* .
* rosrun, roslaunchの前にsource
* Merge branch 'develop'
* C++20 -> C++23
* impl: HITL
* バックアップファイルをROSパッケージの外に出した
* ユーザ用Pythonパッケージを作成
* .
* add: subprocess.xpp
* ros1 -> ros2: simulation
* Merge branch 'develop'
* アイコンを/usr/share/icons/以下に配置
* clean workspace
* fix
* --merge-install
* linux依存を減らした
* Merge remote-tracking branch 'origin/feature/ros1_to_ros2' into develop
* fix
* wip: tobas_gui
* add: parameter_tuning_gui
* add icon
* add: tobas_gui_common (std::formatを使うため)
* Contributors: Masayoshi Dohi, dohi, ok10171
