from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gcs import GroundControlStationWidget

import os.path as osp
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_std_tools_py.config_parser import ConfigParserWrapper
from tobas_rqt_tools.widgets import Widget, ProgressDialog
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import CONFIG_PATH
from tobas_tools_py.drone import Drone, DroneLoader_File

from .common import *
from .utils.ssh_client import SSHClientWrapper


class PackageManagerWidget(Widget):
    KEY = "last_opened_dir/tobas_configuration_package"

    PATH_WIDTH = 300
    BUTTON_WIDTH = 50

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        self._config = ConfigParserWrapper(CONFIG_PATH, PKG_NAME)
        self._ssh_client = SSHClientWrapper()

        cols = QHBoxLayout()
        self.setLayout(cols)

        label = QLabel("Tobas Package Path:")
        cols.addWidget(label)

        self._pkg_path = QLineEdit()
        self._pkg_path.setFixedWidth(self.PATH_WIDTH)
        self._pkg_path.setReadOnly(True)
        self._pkg_path.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._pkg_path)

        self._load_button = QPushButton("Load")
        self._load_button.setFixedWidth(self.BUTTON_WIDTH)
        self._load_button.clicked.connect(self._on_load_button_clicked)
        cols.addWidget(self._load_button)

        self._send_button = QPushButton("Send")
        self._send_button.setFixedWidth(self.BUTTON_WIDTH)
        self._send_button.setEnabled(False)
        self._send_button.clicked.connect(self._on_send_button_clicked)
        cols.addWidget(self._send_button)

    def package_path(self) -> str:
        return self._pkg_path.text()

    def _load_drone(self, pkg_path: str) -> bool:
        """TobasパッケージからDroneをロード．"""
        # TBSFファイルが存在することを確認
        tbsf_path = osp.join(pkg_path, "config/drone.tbsf")
        if not osp.isfile(tbsf_path):
            return False

        # TBSFファイルが正常に読み込めることを確認
        try:
            DroneLoader_File(self._drone, tbsf_path).load()
        except Exception as e:
            rospy.logerr(f"Failed to load TBSF:\n\n{e}")
            return False

        return True

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read()  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(self.KEY, fallback=osp.expanduser("~"))

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
        if not self._load_drone(pkg_path):
            q_error(self._main, f'"{pkg_path}" is not a Tobas configuration package or is collapsed.')
            return

        # パスをテキストに設定
        self._pkg_path.setText(pkg_path)

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        self._config.set(self.KEY, osp.dirname(pkg_path))
        self._config.write()

        # Writeボタンを有効化
        self._send_button.setEnabled(True)

        # 内部状態を更新
        self._main.update_internal_data_structures()

        # ロードが成功したことを示すダイアログ
        q_info(self._main, "Tobas configuration package is loaded successfully.")

    @pyqtSlot()
    def _on_send_button_clicked(self) -> None:
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

        pkg_path = self.package_path()
        pkg_name = pkg_path.split("/")[-1]

        # Tobasパッケージを送信
        # FIXME: メッシュファイルを送るのに多大な時間がかかる．ラズパイ側では不要だから省略したい．
        progress.setLabelText("Sending Tobas configuration package.")
        try:
            self._ssh_client.scp_put_super(pkg_path, osp.join(CATKIN_WS_TOBAS, "src/"))
        except Exception as e:
            progress.close()
            q_error(self._main, f"Failed to send tobas configuration package:\n\n{e}")
            return
        progress.progress_step()

        # Tobasパッケージをビルド
        # NOTE: Paramikoは非対話型セッションを開始するため，コマンドごとに必要な環境変数を設定する．
        # TODO: ビルド時間が長いため，PCでコンパイルしてから実行に必要なファイルのみを送る．
        progress.setLabelText("Building Tobas configuration package.")
        command = SOURCE_CMD + f" && cd {CATKIN_WS_TOBAS} && catkin build {pkg_name}"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:  # ビルドできなければcatkin cleanして再試行
            rospy.logwarn("Failed to build. Retrying...")
            command = SOURCE_CMD + f" && cd {CATKIN_WS_TOBAS} && catkin clean -y && catkin build {pkg_name}"
            success, _, error_output = self._ssh_client.exec_command_super(command)
            if not success:
                progress.close()
                q_error(self._main, f"Failed to build the Tobas configuration package:\n\n{error_output}")
                return
        progress.progress_step()

        # 環境変数TOBAS_CONFIG_PKGを設定
        progress.setLabelText("Setting environment variables.")
        try:
            self._ssh_client.sftp_write_super("/etc/tobas/config_pkg.env", f"TOBAS_CONFIG_PKG={pkg_name}\n")
        except Exception as e:
            progress.close()
            q_error(self._main, str(e))
            return
        progress.progress_step()

        # ROS関連の全てのサービスを再起動 (でないと古いトピックが残ってしまう)
        progress.setLabelText("Restarting Tobas software.")
        command = "systemctl restart tobas_roscore.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            progress.close()
            q_error(self._main, f"Failed to restart Tobas software:\n\n{error_output}")
            return
        progress.progress_step()

        # リロード
        progress.setLabelText("Reloading.")
        self._main.update_internal_data_structures()
        progress.progress_step()

        progress.close()
        q_info(self._main, "Tobas configuration package is installed successfully.")
