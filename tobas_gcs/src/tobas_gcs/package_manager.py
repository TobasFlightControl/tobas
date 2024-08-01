from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gcs import GroundControlStationWidget

import os.path as osp
import rclpy
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QLabel, QLineEdit, QPushButton, QFileDialog, QHBoxLayout

from tobas_property_tools_py.property_client import PropertyClient
from tobas_rqt_tools.widgets import Widget, ProgressDialog
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import PROPERTY_SERVER_GCS, PKG_EXTENSION
from tobas_tools_py.drone import Drone, DroneLoader_File
from tobas_tools_py.package import (
    get_tbs_meta_name,
    get_tbs_config_name,
    get_tbsdrn_path,
    get_mesh_path,
)

from .common import TITLE, PKG_NAME, CATKIN_WS_TOBAS, SOURCE_CMD
from .utils.ssh_client import SSHClientWrapper


class PackageManagerWidget(Widget):
    LAST_OPENED_DIR_KEY = "last_opened_dir/tobas_configuration_package"

    PATH_WIDTH = 300
    BUTTON_WIDTH = 50

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        self._property_client = PropertyClient(PROPERTY_SERVER_GCS, PKG_NAME)
        self._ssh_client = SSHClientWrapper()

        cols = QHBoxLayout()
        self.setLayout(cols)

        label = QLabel("Tobas Package Path:")
        cols.addWidget(label)

        self._tbs_path = QLineEdit()
        self._tbs_path.setFixedWidth(self.PATH_WIDTH)
        self._tbs_path.setReadOnly(True)
        self._tbs_path.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._tbs_path)

        self._load_button = QPushButton("Load")
        self._load_button.setFixedWidth(self.BUTTON_WIDTH)
        self._load_button.clicked.connect(self._on_load_button_clicked)
        cols.addWidget(self._load_button)

        self._send_button = QPushButton("Send")
        self._send_button.setFixedWidth(self.BUTTON_WIDTH)
        self._send_button.setEnabled(False)
        self._send_button.clicked.connect(self._on_send_button_clicked)
        cols.addWidget(self._send_button)

    def tbs_path(self) -> str:
        return self._tbs_path.text()

    def _load_drone(self, tbs_path: str) -> bool:
        """TobasパッケージからDroneをロード．"""
        # TBSFファイルが存在することを確認
        tbsf_path = get_tbsdrn_path(tbs_path)
        if not osp.isfile(tbsf_path):
            q_error(self._main, f"{tbsf_path} does not exist.")
            return False

        # TBSFファイルが正常に読み込めることを確認
        try:
            DroneLoader_File(self._drone, tbsf_path).load()
        except Exception as e:
            q_error(self, f"Failed to load TBSF:\n\n{e}")
            return False

        return True

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        res, last_opened_dir = self._property_client.get_string(self.LAST_OPENED_DIR_KEY)
        if res < 0:
            rclpy.logwarn(self._property_client.error_message())
            last_opened_dir = osp.expanduser("~")

        # Tobasパッケージのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.ShowDirsOnly
        options |= QFileDialog.DontResolveSymlinks
        tbs_path = QFileDialog.getExistingDirectory(self, TITLE, last_opened_dir, options=options)
        assert not tbs_path.endswith("/")  # NOTE: スラッシュで終わる場合はosp.dirname, osp.basename等の挙動が変わる

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if tbs_path == "":
            return

        # 拡張子をチェック
        if not tbs_path.endswith(PKG_EXTENSION):
            q_error(
                self._main,
                f'"{tbs_path}" is not a Tobas configuration package (*{PKG_EXTENSION}).',
            )
            return

        # ドローンの機体情報を読み込む
        if not self._load_drone(tbs_path):
            return

        # パスをテキストに設定
        self._tbs_path.setText(tbs_path)

        # ユーザが開いたディレクトリを保存
        if self._property_client.set_string(self.LAST_OPENED_DIR_KEY, osp.dirname(tbs_path)) < 0:
            self.get_logger().error(self._property_client.error_message())
        if self._property_client.save() < 0:
            self.get_logger().error(self._property_client.error_message())

        # Writeボタンを有効化
        self._send_button.setEnabled(True)

        # 内部状態を更新
        self._main.update_internal_data_structures()

        # ロードが成功したことを示すダイアログ
        q_info(self._main, "Tobas configuration package is loaded successfully.")

    @pyqtSlot()
    def _on_send_button_clicked(self) -> None:
        tbs_path = self.tbs_path()

        progress = ProgressDialog(parent=self._main, title=TITLE, num_steps=6)
        progress.setCancelButton(None)
        progress.show()

        # SSH接続
        progress.setLabelText("Connecting to the Raspberry Pi.")
        try:
            self._ssh_client.connect()
        except Exception as e:
            progress.close()
            q_error(self._main, str(e))
            return
        progress.progress_step()

        # Tobasパッケージを送信
        # FIXME: メッシュファイルを送るのに多大な時間がかかる．ラズパイ側では不要だから省略したい．
        progress.setLabelText("Sending Tobas configuration package.")
        mesh_path = get_mesh_path(tbs_path)
        remote_dir = osp.join(CATKIN_WS_TOBAS, "src/")
        try:
            self._ssh_client.scp_put_dir_super(tbs_path, remote_dir, exclude_dir=mesh_path)
        except Exception as e:
            progress.close()
            q_error(self._main, f"Failed to send tobas configuration package:\n\n{e}")
            return
        progress.progress_step()

        # Tobasパッケージをビルド
        # NOTE: Paramikoは非対話型セッションを開始するため，コマンドごとに必要な環境変数を設定する．
        # TODO: ビルド時間が長いため，PCでコンパイルしてから実行に必要なファイルのみを送る．
        progress.setLabelText("Building Tobas configuration package.")
        meta_pkg_name = get_tbs_meta_name(tbs_path)
        command = SOURCE_CMD + f" && cd {CATKIN_WS_TOBAS} && catkin build {meta_pkg_name}"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:  # ビルドできなければcatkin cleanして再試行
            rclpy.logwarn("Failed to build. Retrying...")
            command = SOURCE_CMD + f" && cd {CATKIN_WS_TOBAS} && catkin clean -y && catkin build {meta_pkg_name}"
            success, _, error_output = self._ssh_client.exec_command_super(command)
            if not success:
                progress.close()
                q_error(
                    self._main,
                    f"Failed to build the Tobas configuration package:\n\n{error_output}",
                )
                return
        progress.progress_step()

        # 環境変数TOBAS_CONFIG_PKGを設定
        progress.setLabelText("Setting environment variables.")
        try:
            self._ssh_client.sftp_write_super(
                "/etc/tobas/config_pkg.env",
                f"TOBAS_CONFIG_PKG={get_tbs_config_name(self._main.tbs_path())}\n",
            )
        except Exception as e:
            progress.close()
            q_error(self._main, str(e))
            return
        progress.progress_step()

        # ROS関連の全てのサービスを再起動 (でないと古いトピックが残ってしまう)
        progress.setLabelText("Restarting Tobas.")
        command = "systemctl restart tobas_main.target"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            progress.close()
            q_error(self._main, f"Failed to restart Tobas:\n\n{error_output}")
            return
        progress.progress_step()

        # リロード
        progress.setLabelText("Reloading.")
        self._main.update_internal_data_structures()
        progress.progress_step()

        progress.close()
        q_info(self._main, "Tobas configuration package is installed successfully.")
