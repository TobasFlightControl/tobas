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
from tobas_calibration_msgs.msg import EscCalibrationAction, EscCalibrationGoal, EscCalibrationResult


from ....common import *
from .base import BaseHardwareSetupWidget


class EscCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "ESC Calibration"
    TITLE = "Calibrate ESCs"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

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

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._start_button.clicked.connect(self._on_start_button_clicked)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        esc_calib_ac = actionlib.SimpleActionClient(f"/{self._drone.drone_name}/esc_calibration", EscCalibrationAction)

        try:
            esc_calib_ac.wait_for_server(rospy.Duration(self.WAIT_FOR_SERVICE))
        except rospy.ROSException:
            q_error(self, "Failed to connect to the calibration server.")
            return

        goal = EscCalibrationGoal()
        esc_calib_ac.send_goal_and_wait(goal)

        if esc_calib_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            q_error(self, esc_calib_ac.get_goal_status_text())
            return

        q_info(self._main, "ESC calibration finished.")
