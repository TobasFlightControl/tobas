from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from tobas_calibration_msgs.srv import AccelCalibration, AccelCalibrationRequest, AccelCalibrationResponse

from ....common import *
from .base import BaseHardwareSetupWidget


class AccelCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Accel Calibration"
    TITLE = "Calibrate Accelerometer"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        instruction = Description('Press "Start" button with the flight controller\'s TOP surface facing up.\n\n')
        self._rows.addWidget(instruction)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._rows.addWidget(self._start_button)

        # TODO: Rvizで重力方向と測定結果を表示

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._start_button.clicked.connect(self._on_start_button_clicked)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        accel_calib_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/accel_calibration", AccelCalibration)

        try:
            accel_calib_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, "Failed to connect to the calibration server.")
            return

        req = AccelCalibrationRequest()
        res: AccelCalibrationResponse = accel_calib_sc.call(req)

        if not res.success:
            q_error(self, res.message)
            return

        q_info(self._main, "Accel calibration finished.")
