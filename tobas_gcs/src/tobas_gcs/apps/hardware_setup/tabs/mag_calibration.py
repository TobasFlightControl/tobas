from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os.path as osp
import rospy
import rospkg
from overrides import override
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *



from tobas_rqt_tools.widgets import create_rviz_frame
from tobas_rqt_tools.messages import *
from tobas_calibration_msgs.srv import MagCalibration, MagCalibrationRequest, MagCalibrationResponse

from ....common import *
from .base import BaseHardwareSetupWidget


class MagCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Magnet Calibration"
    TITLE = "Calibrate Magnetometer"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        instruction = Description(
            '1. Press "Start" button.\n\n'
            "2. For each of the 6 faces of the FC, "
            "slowly rotate the FC twice around the direction of gravity with the face pointing upwards.\n\n"
            "3. Confirm that the point cloud forms a neat ellipsoid on the screen below.\n\n"
            '4. Press "Finish" button.\n\n'
        )
        self._rows.addWidget(instruction)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._start_button)

        self._finish_button = QPushButton("Finish")
        self._finish_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._finish_button.setEnabled(False)
        cols.addWidget(self._finish_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        cols.addWidget(self._cancel_button)

        cols.addStretch()

        pkg_path = rospkg.RosPack().get_path(PKG_NAME)
        rviz_config_path = osp.join(pkg_path, "config/mag_calibration.rviz")
        self._rviz_frame = create_rviz_frame(rviz_config_path)
        self._rows.addWidget(self._rviz_frame)

        self._rows.addStretch()

        self._calib_start_sc = rospy.ServiceProxy("/mag_calibration/start", Trigger)
        self._calib_finish_sc = rospy.ServiceProxy("/mag_calibration/finish", MagCalibration)
        self._calib_cancel = rospy.ServiceProxy("/mag_calibration/cancel", Trigger)

    @override
    def define_connections(self) -> None:
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._finish_button.clicked.connect(self._on_finish_button_clicked)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        try:
            self._calib_start_sc.wait_for_service(self.WAIT_FOR_SERVICE)
        except rospy.ROSException:
            q_error(self, "Failed to connect to the calibration server.")
            return

        req = TriggerRequest()
        res: TriggerResponse = self._calib_start_sc.call(req)

        if not res.success:
            q_error(self, res.message)
            return

        self._start_button.setEnabled(False)
        self._finish_button.setEnabled(True)
        self._cancel_button.setEnabled(True)

        q_info(self._main, "Magnet calibration started.")

    @pyqtSlot()
    def _on_finish_button_clicked(self) -> None:
        try:
            self._calib_finish_sc.wait_for_service(self.WAIT_FOR_SERVICE)
        except rospy.ROSException:
            q_error(self, "Failed to connect to the calibration server.")
            return

        req = MagCalibrationRequest()
        res: MagCalibrationResponse = self._calib_finish_sc.call(req)

        if not res.success:
            q_error(self, res.message)
            return

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

        q_info(self._main, "Magnet calibration finished.")

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        try:
            self._calib_cancel.wait_for_service(self.WAIT_FOR_SERVICE)
        except rospy.ROSException:
            q_error(self, "Failed to connect to the calibration server.")
            return

        req = TriggerRequest()
        res: TriggerResponse = self._calib_cancel.call(req)

        if not res.success:
            q_error(self, res.message)
            return

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

        q_info(self._main, "Magnet calibration is cancelled.")
