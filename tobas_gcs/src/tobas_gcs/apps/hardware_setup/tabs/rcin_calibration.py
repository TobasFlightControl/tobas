from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rclpy
from typing import override
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout

from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.widgets import HPositionBarWidget, VPositionBarWidget
from tobas_rqt_tools.utils import place_center, create_fixed_height_hboxlayout
from tobas_tools_py.constants import RCChannel, Service
from tobas_tools_py.drone import Drone
from tobas_hal_msgs.msg import Sbus
from tobas_calibration_msgs.srv import (
    RCInputCalibration,
    RCInputCalibrationRequest,
    RCInputCalibrationResponse,
)

from ....common import WAIT_FOR_SERVER, Description
from .base import BaseHardwareSetupWidget


class RcinCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Radio Calibration"
    TITLE = "Calibrate RC Input"

    PWM_MIN = 900
    PWM_MAX = 2100
    RANGE_SIDE_SHORT = 30
    RANGE_SIDE_LONG = 300

    MODE_TEXT = "Program" + " " * 15 + "Stabilize" + " " * 15 + "Acrobat"
    ON_OFF_TEXT = "ON" + " " * 55 + "OFF"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        instruction = Description(
            '1. Press "Start" button.\n\n'
            "2. For each channel, operate the stick or switch to ensure it covers the entire range. "
            "If the stick's movement is opposite to that of the bar, adjust the transmitter settings accordingly.\n\n"
            '3. Press "Finish" button.\n\n'
        )
        self._rows.addWidget(instruction)

        cols1 = QHBoxLayout()
        self._rows.addLayout(cols1)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._start_button.clicked.connect(self._on_start_button_clicked)
        cols1.addWidget(self._start_button)

        self._finish_button = QPushButton("Finish")
        self._finish_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._finish_button.setEnabled(False)
        self._finish_button.clicked.connect(self._on_finish_button_clicked)
        cols1.addWidget(self._finish_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)
        cols1.addWidget(self._cancel_button)

        cols1.addStretch()

        self._rows.addSpacing(50)

        cols2 = QHBoxLayout()
        self._rows.addLayout(cols2)

        # Roll, Pitch, Yaw, Throttle
        cols3 = create_fixed_height_hboxlayout(self.RANGE_SIDE_LONG + 20, cols2)

        self._pitch_range = VPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._pitch_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols3.addWidget(self._pitch_range)

        rows1 = QVBoxLayout()
        cols3.addLayout(rows1)

        self._roll_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._roll_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._roll_range, rows1)
        place_center(QLabel(f"Roll (CH{RCChannel.ROLL + 1})"), rows1)

        rows1.addStretch()

        cols4 = QHBoxLayout()
        rows1.addLayout(cols4)

        pitch_label = QLabel(f"Pitch (CH{RCChannel.PITCH + 1})")
        pitch_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        cols4.addWidget(pitch_label)

        throttle_label = QLabel(f"Throttle (CH{RCChannel.THROTTLE + 1})")
        throttle_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        cols4.addWidget(throttle_label)

        rows1.addStretch()

        place_center(QLabel(f"Yaw (CH{RCChannel.YAW + 1})"), rows1)
        self._yaw_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._yaw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._yaw_range, rows1)

        self._throttle_range = VPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._throttle_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols3.addWidget(self._throttle_range)

        cols2.addSpacing(30)

        bar_grid = QGridLayout()
        cols2.addLayout(bar_grid)

        # Mode
        bar_grid.addWidget(QLabel(f"Mode (CH{RCChannel.MODE + 1})"), 0, 0)
        self._mode_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._mode_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._mode_range, 0, 1)

        # E-Stop
        bar_grid.addWidget(QLabel(f"E-Stop (CH{RCChannel.ESTOP + 1})"), 1, 0)
        self._estop_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._estop_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._estop_range, 1, 1)

        # GPSw
        bar_grid.addWidget(QLabel(f"GPSw (CH{RCChannel.GPSW + 1})"), 2, 0)
        self._gpsw_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._gpsw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._gpsw_range, 2, 1)

        cols2.addStretch()
        self._rows.addStretch()

        self._sbus_sub: self.create_subscription = None

        self._reset()

        self.setEnabled(False)

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)

    def _reset(self) -> None:
        if self._sbus_sub is not None:
            self._sbus_sub.unregister()

        self._roll_range.clear()
        self._pitch_range.clear()
        self._yaw_range.clear()
        self._throttle_range.clear()
        self._mode_range.clear()
        self._estop_range.clear()
        self._gpsw_range.clear()

        self._mode_range.set_text(self.MODE_TEXT)
        self._estop_range.set_text(self.ON_OFF_TEXT)
        self._gpsw_range.set_text(self.ON_OFF_TEXT)

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

    def _sbus_cb(self, sbus: Sbus) -> None:
        self._roll_range.set_value(sbus.data[RCChannel.ROLL])
        self._pitch_range.set_value(sbus.data[RCChannel.PITCH])
        self._yaw_range.set_value(sbus.data[RCChannel.YAW])
        self._throttle_range.set_value(sbus.data[RCChannel.THROTTLE])
        self._mode_range.set_value(sbus.data[RCChannel.MODE])
        self._estop_range.set_value(sbus.data[RCChannel.ESTOP])
        self._gpsw_range.set_value(sbus.data[RCChannel.GPSW])

        self._roll_range.update()
        self._pitch_range.update()
        self._yaw_range.update()
        self._throttle_range.update()
        self._mode_range.update()
        self._estop_range.update()
        self._gpsw_range.update()

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        # S.BUSトピックが正常に発行されていることを確認
        sbus_topic = f"{self._drone.name}/hal/sbus"
        try:
            rclpy.wait_for_message(sbus_topic, Sbus, WAIT_FOR_SERVER)
        except Exception:
            q_error(self, f"Failed to get RC input message in {WAIT_FOR_SERVER} seconds.")
            self._reset()
            return

        # 一時的にS.BUSトピックを購読開始
        self._sbus_sub = self.create_subscription(sbus_topic, Sbus, self._sbus_cb, 1)

        self._start_button.setEnabled(False)
        self._finish_button.setEnabled(True)
        self._cancel_button.setEnabled(True)

        q_info(self._main, "Radio calibration started.")

    @pyqtSlot()
    def _on_finish_button_clicked(self) -> None:
        if not self._finish_calibration():
            return

        if not self._reload_config():
            return

        self._reset()
        q_info(self._main, "Radio calibration finished.")

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        self._reset()
        q_info(self._main, "Radio calibration is cancelled.")

    def _finish_calibration(self) -> bool:
        calib_sc = self._node.create_client(f"{self._drone.name}/rcin_calibration", RCInputCalibration)
        try:
            calib_sc.service_is_ready(WAIT_FOR_SERVER)
        except rclpy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = RCInputCalibrationRequest()
        req.roll_left = int(self._roll_range.get_lower())
        req.roll_right = int(self._roll_range.get_upper())
        req.pitch_down = int(self._pitch_range.get_upper())
        req.pitch_up = int(self._pitch_range.get_lower())
        req.yaw_left = int(self._yaw_range.get_lower())
        req.yaw_right = int(self._yaw_range.get_upper())
        req.throttle_down = int(self._throttle_range.get_upper())
        req.throttle_up = int(self._throttle_range.get_lower())
        req.mode_program = int(self._mode_range.get_lower())
        req.mode_stabilize = int(self._mode_range.get_middle())
        req.mode_acrobat = int(self._mode_range.get_upper())
        req.estop_on = int(self._estop_range.get_lower())
        req.estop_off = int(self._estop_range.get_upper())
        req.gpsw_on = int(self._gpsw_range.get_lower())
        req.gpsw_off = int(self._gpsw_range.get_upper())

        try:
            res: RCInputCalibrationResponse = calib_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True

    def _reload_config(self) -> bool:
        reload_config_sc = self._node.create_client(f"{self._drone.name}/rcin_handler/{Service.RELOAD_CONFIG}", Trigger)
        try:
            reload_config_sc.service_is_ready(WAIT_FOR_SERVER)
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
