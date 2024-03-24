from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gcs import GroundControlStationWidget

import os.path as osp
import rospy
from configparser import ConfigParser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone, DroneLoader_File

from .common import *
from .utils.ssh_client import SSHClientWrapper


class PackageManagerWidget(QWidget):
    KEY = "last_opened_dir/tobas_configuration_package"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
        self._main = main

        self._config = ConfigParser()
        self._ssh_client = SSHClientWrapper()

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
            q_error(self._main, f'"{pkg_path}" is not a Tobas configuration package or is collapsed.')
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
        rospy.loginfo("Connecting to the Raspberry Pi.")
        try:
            self._ssh_client.connect()
        except Exception as e:
            q_error(self._main, str(e))
            return

        pkg_path = self._pkg_path.text()
        pkg_name = pkg_path.split("/")[-1]

        # Tobasパッケージを送信
        rospy.loginfo("Sending Tobas configuration package.")
        self._ssh_client.scp_put(pkg_path, osp.join(CATKIN_WS_TOBAS, "src/"))

        # ビルド
        rospy.loginfo("Building Tobas configuration package.")
        command = f"cd {CATKIN_WS_TOBAS} && catkin build {pkg_name}"
        success, _, error_output = self._ssh_client.exec_command(command)
        if not success:
            q_error(self._main, f"Failed to build the Tobas configuration package:\n\n{error_output}")
            return

        # 環境変数TOBAS_CONFIG_PKGを設定
        rospy.loginfo("Setting environment variables")
        try:
            self._ssh_client.sftp_write_super("/etc/tobas/config_pkg.env", f"TOBAS_CONFIG_PKG={pkg_name}\n")
        except Exception as e:
            q_error(self._main, str(e))
            return

        # サービスを再起動
        rospy.loginfo("Restarting tobas_real.service.")
        command = "systemctl restart tobas_real.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to restart Tobas software:\n\n{error_output}")
            return

        q_info(self._main, "Tobas configuration package is installed successfully.")
