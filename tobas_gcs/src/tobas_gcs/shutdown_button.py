from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gcs import GroundControlStationWidget

import os
import signal
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_error, yes_or_no, QMessageLevel
from tobas_tools_py.drone import Drone

from .utils.ssh_client import SSHClientWrapper


class ShutdownButtonWidget(QPushButton):

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        self.setText("Shutdown")
        self.setStyleSheet("background-color: red")
        self.clicked.connect(self._on_clicked)

        self._ssh_client = SSHClientWrapper()

    @pyqtSlot()
    def _on_clicked(self) -> None:
        # 本当にシャットダウンしてよいか確認
        if not yes_or_no(self._main, "Are you sure you want to shut down the FC and the GCS?", QMessageLevel.WARN):
            return

        # SSH接続
        rospy.loginfo("Connecting to the Raspberry Pi.")
        try:
            self._ssh_client.connect()
        except Exception as e:
            q_error(self._main, str(e))
            return

        # ラズパイをシャットダウン
        rospy.loginfo("Shutting down the Raspberry Pi.")
        self._ssh_client.exec_command_bg_super("poweroff")

        # GCSを強制終了
        os.kill(os.getpid(), signal.SIGINT)
