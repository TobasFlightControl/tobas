from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os.path as osp
import rospy
import rospkg
from overrides import override
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QHBoxLayout

from tobas_rqt_tools.rviz import create_rviz_frame
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone
from tobas_calibration_msgs.srv import MagCalibration, MagCalibrationRequest, MagCalibrationResponse

from ....common import PKG_NAME, WAIT_FOR_SERVER, Description
from .base import BaseHardwareSetupWidget


class MagCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Magnet Calibration"
    TITLE = "Calibrate Magnetometer"

    POINT_HISTORY_LENGTH = 100000

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

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

        manager = self._rviz_frame.getManager()
        display = manager.getRootDisplayGroup().getDisplayAt(0)
        assert display.getName() == "PointStamped"
        self._point_topic = display.subProp("Topic")
        self._point_history_length = display.subProp("History Length")

        self._rows.addStretch()

        self.setEnabled(False)

    @override
    def define_connections(self) -> None:
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._finish_button.clicked.connect(self._on_finish_button_clicked)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        # Rvizのトピックを変更
        self._point_topic.setValue(f"{self._drone.drone_name}/mag_calibration/magnetic_field_raw")

        self.setEnabled(True)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        calib_start_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/mag_calibration/start", Trigger)

        try:
            calib_start_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return

        req = TriggerRequest()

        try:
            res: TriggerResponse = calib_start_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if not res.success:
            q_error(self, res.message)
            return

        self._point_history_length.setValue(self.POINT_HISTORY_LENGTH)

        self._start_button.setEnabled(False)
        self._finish_button.setEnabled(True)
        self._cancel_button.setEnabled(True)

        q_info(self._main, "Magnet calibration started.")

    @pyqtSlot()
    def _on_finish_button_clicked(self) -> None:
        if not self._finish_calibration():
            return

        if not self._reload_config():
            return

        self._point_history_length.setValue(0)

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

        q_info(self._main, "Magnet calibration finished.")

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        calib_cancel_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/mag_calibration/cancel", Trigger)
        try:
            calib_cancel_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return

        try:
            res: TriggerResponse = calib_cancel_sc.call(TriggerRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if not res.success:
            q_error(self, res.message)
            return

        self._point_history_length.setValue(0)

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

        q_info(self._main, "Magnet calibration is cancelled.")

    def _finish_calibration(self) -> bool:
        calib_finish_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/mag_calibration/finish", MagCalibration)

        try:
            calib_finish_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = MagCalibrationRequest()

        try:
            res: MagCalibrationResponse = calib_finish_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True

    def _reload_config(self) -> bool:
        reload_config_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/imu_handler/reload_config", Trigger)
        try:
            reload_config_sc.wait_for_service(WAIT_FOR_SERVER)
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
