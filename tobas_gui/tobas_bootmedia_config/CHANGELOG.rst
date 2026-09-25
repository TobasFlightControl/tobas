^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tobas_bootmedia_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* modify
* modify
* 依存をpackage.xmlへ
* Add description
* サブモジュールを削除してベンダーパッケージに
* magic_enum.hpp -> magic_enum/magic_enum.hpp
* magic_enumとcxxoptsのサブモジュールを削除
* 0.0.0 -> 2.16.0
* NIF -> NIC
* fix
* .
* PasswordEditを周りのセルになじませる．
* パスワードを表示可能に
* .
* Merge branch 'feature/english_translation' into jazzy
* English translation
* Contributors: Masayoshi Dohi

2.15.0 (2026-06-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* C由来のヘッダを排除
* Merge branch 'jazzy-dev' into jazzy
* .
* QOverload -> qOverload
* .
* Merge branch 'feature/fix_ip_address' into jazzy-dev
* fix: swap bite order
* add license headers
* fix
* implement IPv4 settings
* .
* Connectの時点で全項目Read
* Disconnectでデバイスの取り外しまで行う
* Merge branch 'jazzy-dev' into jazzy
* add license
* add license
* Merge branch 'feature/add_namespace' into jazzy-dev
* delete unecessary namespace specifications
* .
* add namespace
* Merge branch 'jazzy-dev' into feature/objetc_avoidance
* add namespace
* QDebugに着色
* Merge branch 'jazzy-dev' into jazzy
* add compile option: -Wswitch-enum
* tobas_constantsをファイル分割
* Merge branch 'feature/update_raspbian_version' into jazzy-dev
* wpa_supplicant.conf -> wpa_supplicant-nl80211-wlan0.conf
* Merge branch 'release/v2.12' into feature/refine_img_processing
* Contributors: Masayoshi Dohi, Tetsuka Daiki, ok10171

2.12.1 (2026-02-18 12:13:14 +0900)
----------------------------------

2.12.0 (2026-02-18 12:13:14 +0900)
----------------------------------
* Merge branch 'jazzy-dev' into jazzy
* remove unnecessary installation
* Contributors: Masayoshi Dohi

2.11.0 (2026-02-13)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* fix: qDebug
* 不要な定義を削除
* Contributors: Masayoshi Dohi

2.10.0 (2026-01-29)
-------------------
* Merge branch 'jazzy-dev' into jazzy
* .
* modify
* Merge branch 'jazzy-dev' into jazzy
* modify: size() > 0 -> !empty()
* tobas_std -> tbs
* change: SAとGCSでプロジェクトのバージョンチェック
* change: バージョンファイルを保存
* modify: GUIのタイトルにバージョンを表示
* modify: KeyMgmtにWPA3-Personalを追加
* Contributors: Masayoshi Dohi

2.8.0 (2025-09-25)
------------------
* Merge branch 'jazzy-dev' into jazzy
* edit cmakes
* .
* Contributors: Masayoshi Dohi

2.7.0 (2025-09-09)
------------------
* Merge branch 'jazzy-develop' into jazzy
* modify: パス作成時のエラーハンドリング
* modify: gui::common -> gui::cmn
* modify: 典型的な文字サイズをtobas_gui_commonにまとめた
* modify: 不要なインストール，エクスポートを削除
* .
* コンパイルオプションを追加
* Delete OS checks
* change namespace: tobas::sak:: -> tobas::ssh::ak::
* fix: Create authorized_keys file if not exist
* wip: Remote Connection
* Merge branch 'feature/network_settings' into jazzy-develop
* edit: main.cpp
* modify: 無効な文字を拒否
* modify: 接続中にアプリを落とそうとしたら警告
* modify: Reset PasswordEdit
* add: SSH Keys
* .
* edit: tobas_wpa_supplicant
* move: wpa_supplicant_parser
* tobas名前空間に入れた
* add: tobas_crypt
* add: Login password setting widget
* add: hostname setting widget
* PSKの最小文字数を設定
* edit: wifi_hotspot
* fix
* wip: wifi_hotspot
* modify: clearボタンを追加
* pskを隠す
* modify: CMakeLists.txt
* CMAKE_AUTOMOCを活用
* wip: パスワードを隠す
* wip: wifi_client
* wip: wifi_client
* rename
* wip: wifi_client
* edit: wifi_client
* wip
* wip
* wip
* .
* rename: wifi_sta -> wifi_client
* add: Add tobas_bootmedia_config
* Contributors: Masayoshi Dohi
