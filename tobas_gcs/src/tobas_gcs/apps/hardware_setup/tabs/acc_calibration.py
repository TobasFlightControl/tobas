from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone
from tobas_calibration_msgs.srv import AccelCalibration, AccelCalibrationRequest, AccelCalibrationResponse

from ....common import *
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
        self._rows.addWidget(self._start_button)

        # TODO: Rvizで重力方向と測定結果を表示

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        self._start_button.clicked.connect(self._on_start_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        if not self._calibrate():
            return

        if not self._reload_config():
            return

        q_info(self._main, "Accel calibration finished.")

    def _calibrate(self) -> bool:
        accel_calib_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/accel_calibration", AccelCalibration)
        try:
            accel_calib_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
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
        reload_config_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/imu_handler/reload_config", Trigger)
        try:
            reload_config_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
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
