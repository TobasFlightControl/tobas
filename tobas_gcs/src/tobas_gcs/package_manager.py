from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gcs import GroundControlStationWidget

import os.path as osp
import paramiko
import socket
from scp import SCPClient
from configparser import ConfigParser
from typing import Tuple
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import *
from tobas_tools_py.drone import Drone, DroneLoader_File

from .common import *


class PackageManagerWidget(QWidget):
    KEY = "last_opened_dir/tobas_configuration_package"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
        self._main = main

        self._config = ConfigParser()

        # SSHクライアント
        # TODO: AutoAddPolicyは脆弱なので，予めサーバーのホストキーをクライアントに登録する
        self._ssh_client = paramiko.SSHClient()
        self._ssh_client.load_system_host_keys()
        self._ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel("Tobas Package Path:"))

        self._pkg_path = QLineEdit()
        self._pkg_path.setReadOnly(True)
        self._pkg_path.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._pkg_path)

        self._load_button = QPushButton("Load")
        cols.addWidget(self._load_button)

        self._send_button = QPushButton("Send")
        self._send_button.setEnabled(False)
        cols.addWidget(self._send_button)

    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._send_button.clicked.connect(self._on_send_button_clicked)

    def _is_valid_tobas_pkg(self, pkg_path: str) -> bool:
        """有効なTobasパッケージかどうかを判定する．"""
        # TBSFファイルが存在することを確認
        tbsf_path = osp.join(pkg_path, "config/drone.tbsf")
        if not osp.isfile(tbsf_path):
            return False

        # TBSFファイルが正常に読み込めることを確認
        try:
            DroneLoader_File(Drone(), tbsf_path).load()
        except Exception as e:
            rospy.logerr(f"Failed to load TBSF:\n\n{e}")
            return False

        return True

    def _execute_command(self, command: str) -> Tuple[int, str, str]:
        _, stdout, stderr = self._ssh_client.exec_command(command)
        stdout.channel.recv_exit_status()  # コマンドの実行結果を待つ
        output = stdout.read().decode("utf-8")  # 標準出力
        error_output = stderr.read().decode("utf-8")  # 標準エラー出力
        exit_status = stdout.channel.exit_status
        return exit_status, output, error_output

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(DEFAULT, self.KEY, fallback=osp.expanduser("~"))

        # Tobasパッケージのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.ShowDirsOnly
        options |= QFileDialog.DontResolveSymlinks
        pkg_path = QFileDialog.getExistingDirectory(self, TITLE, last_opened_dir, options=options)
        assert not pkg_path.endswith("/")  # NOTE: スラッシュで終わる場合はosp.dirname, osp.basename等の挙動が変わる

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if pkg_path == "":
            return

        # 有効なTobas Configuration Packageでなければ終了
        if not self._is_valid_tobas_pkg(pkg_path):
            q_error(
                self._main,
                f'"{pkg_path}" is not a Tobas configuration package or is collapsed.',
            )
            return

        # パスをテキストに設定
        self._pkg_path.setText(pkg_path)

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        self._config[DEFAULT][self.KEY] = osp.dirname(pkg_path)
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

        # Writeボタンを有効化
        self._send_button.setEnabled(True)

        # Tobasパッケージがロードされたことを通知
        self._main.signals.config_pkg_updated.emit(pkg_path)

        # ロードが成功したことを示すダイアログ
        q_info(self._main, "Tobas configuration package is loaded successfully.")

    @pyqtSlot()
    def _on_send_button_clicked(self) -> None:
        # TODO: 進行状況をダイアログなどで表示

        # SSH接続
        # TODO: SSH鍵認証，環境変数，秘密管理ツール等を使用して認証情報を安全に管理する
        try:
            self._ssh_client.connect(HOST_NAME, PORT, USER, LOGIN_PASSWORD)
        except paramiko.AuthenticationException:
            q_error(
                self._main,
                "Authentication failed. Please check your username or password.",
            )
            return
        except paramiko.SSHException:
            q_error(
                self._main,
                "Failed to establish an SSH connection.",
            )
            return
        except socket.error:
            q_error(
                self._main,
                "Could not connect to the server. Please check your network connection.",
            )
            return
        except Exception as e:
            q_error(self._main, f"Unexpected error occurred:\n\n{e}")
            return

        pkg_path = self._pkg_path.text()
        pkg_name = pkg_path.split("/")[-1]

        # Tobasパッケージを送信
        rospy.loginfo("Sending Tobas configuration package.")
        with SCPClient(self._ssh_client.get_transport()) as scp:
            scp.put(
                pkg_path,
                recursive=True,
                remote_path=osp.join(CATKIN_WS_TOBAS, "src/"),
            )

        # ビルド
        rospy.loginfo("Building Tobas configuration package.")
        command = f"cd {CATKIN_WS_TOBAS} && catkin build {pkg_name}"
        exit_status, _, error_output = self._execute_command(command)
        if exit_status != 0:
            q_error(
                self._main,
                f"Failed to build the Tobas configuration package:\n\n{error_output}",
            )
            return

        # 環境変数TOBAS_CONFIG_PKGを設定
        rospy.loginfo("Setting environment variables")
        command = f'echo "TOBAS_CONFIG_PKG={pkg_name}" | sudo tee /etc/tobas/config_pkg.env > /dev/null'
        exit_status, _, error_output = self._execute_command(command)
        if exit_status != 0:
            q_error(
                self._main,
                f"Failed to set TOBAS_CONFIG_PKG:\n\n{error_output}",
            )
            return

        # サービスを再起動
        rospy.loginfo("Restarting Tobas software.")
        command = SUDO_PREFIX + "systemctl restart tobas_real.service"
        exit_status, _, error_output = self._execute_command(command)
        if exit_status != 0:
            q_error(
                self._main,
                f"Failed to restart Tobas software:\n\n{error_output}",
            )
            return

        q_info(
            self._main,
            "Tobas configuration package is installed successfully.",
        )
