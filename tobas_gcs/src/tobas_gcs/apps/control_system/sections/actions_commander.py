from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os
import signal
import rospy
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_error, yes_or_no, QMessageLevel
from tobas_tools_py.drone import Drone

from ....common import *
from ....utils.ssh_client import SSHClientWrapper
from .base_section import BaseControlSystemSectionWidget


class ActionsCommanderWidget(BaseControlSystemSectionWidget):
    LABEL = "Actions"

    BUTTON_WIDTH = 120
    BUTTON_HEIGHT = 50

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._ssh_client = SSHClientWrapper()

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._takeoff_button = QPushButton("Takeoff")
        self._takeoff_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._takeoff_button)

        self._landing_button = QPushButton("Landing")
        self._landing_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._landing_button)

        self._rth_button = QPushButton("Return to Home")
        self._rth_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._rth_button)

        cols.addStretch()

        self._poweroff_button = QPushButton("Poweroff")
        self._poweroff_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._poweroff_button.setStyleSheet("background-color: red")
        cols.addWidget(self._poweroff_button)

    @override
    def define_connections(self) -> None:
        self._takeoff_button.clicked.connect(self._on_takeoff_button_clicked)
        self._landing_button.clicked.connect(self._on_landing_button_clicked)
        self._rth_button.clicked.connect(self._on_rth_button_clicked)
        self._poweroff_button.clicked.connect(self._on_poweroff_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @pyqtSlot()
    def _on_takeoff_button_clicked(self) -> None:
        q_error(self._main, "Not implemented yet.")  # TODO

    @pyqtSlot()
    def _on_landing_button_clicked(self) -> None:
        q_error(self._main, "Not implemented yet.")  # TODO

    @pyqtSlot()
    def _on_rth_button_clicked(self) -> None:
        q_error(self._main, "Not implemented yet.")  # TODO

    @pyqtSlot()
    def _on_poweroff_button_clicked(self) -> None:
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
        self._ssh_client.exec_command_super("poweroff &")  # 実行結果は帰ってこないためバックグランドで実行

        # GCSを強制終了
        os.kill(os.getpid(), signal.SIGINT)
