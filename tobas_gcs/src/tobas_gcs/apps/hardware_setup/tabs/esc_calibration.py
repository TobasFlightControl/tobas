from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
import actionlib
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone
from tobas_calibration_msgs.msg import EscCalibrationAction, EscCalibrationGoal


from ....common import *
from .base import BaseHardwareSetupWidget


class EscCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "ESC Calibration"
    TITLE = "Calibrate ESCs"

    TIMEOUT = 60  # [s]

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        warning = Description("Warning: Ensure that propellers are removed from motors.\n\n")
        warning.setStyleSheet("color: red; font-weight: bold;")
        self._rows.addWidget(warning)

        instruction = Description(
            "1. Supply power to the Raspberry Pi via a Type-C connection.\n\n"
            "2. Connect the ESCs to the Navio2 in the correct order.\n\n"
            "3. Disconnect the battery from the ESCs.\n\n"
            '4. Press "Start" button.\n\n'
            "5. Connect the battery to the ESCs.\n\n"
        )
        self._rows.addWidget(instruction)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._rows.addWidget(self._start_button)

        self._rows.addStretch()

        self.setEnabled(False)

    @override
    def define_connections(self) -> None:
        self._start_button.clicked.connect(self._on_start_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        esc_calib_ac = actionlib.SimpleActionClient(f"{self._drone.drone_name}/esc_calibration", EscCalibrationAction)
        if not esc_calib_ac.wait_for_server(rospy.Duration(self.WAIT_FOR_SERVER)):
            q_error(self, self.E_FAILED_TO_CONNECT)
            return

        # TODO: アクションの実行中に接続が切れた場合の対応
        esc_calib_ac.send_goal_and_wait(EscCalibrationGoal())
        if esc_calib_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            q_error(self, esc_calib_ac.get_goal_status_text())
            return

        q_info(self._main, "ESC calibration finished.")
