from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rclpy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from overrides import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton

from tobas_rqt_tools.widgets import ProgressDialog
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import Service
from tobas_tools_py.drone import Drone
from tobas_calibration_msgs.srv import (
    AccelCalibration,
    AccelCalibrationRequest,
    AccelCalibrationResponse,
)

from ....common import WAIT_FOR_SERVER, Description
from .base import BaseHardwareSetupWidget


class AccelCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Accel Calibration"
    TITLE = "Calibrate Accelerometer"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        instruction = Description('Press "Start" button with the flight controller\'s TOP surface facing up.\n\n')
        self._rows.addWidget(instruction)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._rows.addWidget(self._start_button)

        # TODO: Rvizで重力方向と測定結果を表示

        self._rows.addStretch()

        self.setEnabled(False)

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        progress = ProgressDialog(parent=self._main, title=self.NAME, num_steps=2)
        progress.setCancelButton(None)
        progress.show()

        progress.setLabelText("Calibrating.")
        if not self._calibrate():
            progress.close()
            return
        progress.progress_step()

        progress.setLabelText("Reloading.")
        if not self._reload_config():
            progress.close()
            return
        progress.progress_step()

        progress.close()
        q_info(self._main, "Accel calibration finished.")

    def _calibrate(self) -> bool:
        accel_calib_sc = rclpy.ServiceProxy(f"{self._drone.name}/accel_calibration", AccelCalibration)
        try:
            accel_calib_sc.wait_for_service(WAIT_FOR_SERVER)
        except rclpy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        try:
            res: AccelCalibrationResponse = accel_calib_sc.call(AccelCalibrationRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True

    def _reload_config(self) -> bool:
        reload_config_sc = rclpy.ServiceProxy(f"{self._drone.name}/imu_handler/{Service.RELOAD_CONFIG}", Trigger)
        try:
            reload_config_sc.wait_for_service(WAIT_FOR_SERVER)
        except rclpy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        try:
            res: TriggerResponse = reload_config_sc.call(TriggerRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True
