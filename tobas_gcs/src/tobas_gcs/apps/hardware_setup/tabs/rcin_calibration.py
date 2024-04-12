from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.widgets import HPositionBarWidget, VPositionBarWidget
from tobas_rqt_tools.utils import place_center, create_fixed_height_hboxlayout
from tobas_tools_py.constants import *
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import RCInputError
from tobas_calibration_msgs.msg import RCInput
from tobas_calibration_msgs.srv import RCInputCalibration, RCInputCalibrationRequest, RCInputCalibrationResponse

from ....common import *
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
        cols1.addWidget(self._start_button)

        self._finish_button = QPushButton("Finish")
        self._finish_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._finish_button.setEnabled(False)
        cols1.addWidget(self._finish_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        cols1.addWidget(self._cancel_button)

        cols1.addStretch()

        self._rows.addSpacing(50)

        cols2 = QHBoxLayout()
        self._rows.addLayout(cols2)

        # Roll, Pitch, Yaw, Thrust
        cols3 = create_fixed_height_hboxlayout(self.RANGE_SIDE_LONG + 20, cols2)

        self._pitch_range = VPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._pitch_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols3.addWidget(self._pitch_range)

        rows1 = QVBoxLayout()
        cols3.addLayout(rows1)

        self._roll_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._roll_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._roll_range, rows1)
        place_center(QLabel(f"Roll (CH{RCIN_ROLL + 1})"), rows1)

        rows1.addStretch()

        cols4 = QHBoxLayout()
        rows1.addLayout(cols4)

        pitch_label = QLabel(f"Pitch (CH{RCIN_PITCH + 1})")
        pitch_label.setAlignment(Qt.AlignLeft)
        cols4.addWidget(pitch_label)

        thrust_label = QLabel(f"Thrust (CH{RCIN_THRUST + 1})")
        thrust_label.setAlignment(Qt.AlignRight)
        cols4.addWidget(thrust_label)

        rows1.addStretch()

        place_center(QLabel(f"Yaw (CH{RCIN_YAW + 1})"), rows1)
        self._yaw_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._yaw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._yaw_range, rows1)

        self._thrust_range = VPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._thrust_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols3.addWidget(self._thrust_range)

        cols2.addSpacing(30)

        bar_grid = QGridLayout()
        cols2.addLayout(bar_grid)

        # Mode
        bar_grid.addWidget(QLabel(f"Mode (CH{RCIN_MODE + 1})"), 0, 0)
        self._mode_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._mode_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._mode_range, 0, 1)

        # E-Stop
        bar_grid.addWidget(QLabel(f"E-Stop (CH{RCIN_ESTOP + 1})"), 1, 0)
        self._estop_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._estop_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._estop_range, 1, 1)

        # GPSw
        bar_grid.addWidget(QLabel(f"GPSw (CH{RCIN_GPSW + 1})"), 2, 0)
        self._gpsw_range = HPositionBarWidget(minimum=self.PWM_MIN, maximum=self.PWM_MAX)
        self._gpsw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._gpsw_range, 2, 1)

        cols2.addStretch()
        self._rows.addStretch()

        self._rcin_sub = None

        self._reset()

    @override
    def define_connections(self) -> None:
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._finish_button.clicked.connect(self._on_finish_button_clicked)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    def _reset(self) -> None:
        self._roll_range.clear()
        self._pitch_range.clear()
        self._yaw_range.clear()
        self._thrust_range.clear()
        self._mode_range.clear()
        self._estop_range.clear()
        self._gpsw_range.clear()

        self._mode_range.set_text(self.MODE_TEXT)
        self._estop_range.set_text(self.ON_OFF_TEXT)
        self._gpsw_range.set_text(self.ON_OFF_TEXT)

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

        if self._rcin_sub is not None:
            self._rcin_sub.unregister()

        self._stop_timers()

    def _start_timers(self) -> None:
        self._roll_range.start_timer()
        self._pitch_range.start_timer()
        self._yaw_range.start_timer()
        self._thrust_range.start_timer()
        self._mode_range.start_timer()
        self._estop_range.start_timer()
        self._gpsw_range.start_timer()

    def _stop_timers(self) -> None:
        self._roll_range.stop_timer()
        self._pitch_range.stop_timer()
        self._yaw_range.stop_timer()
        self._thrust_range.stop_timer()
        self._mode_range.stop_timer()
        self._estop_range.stop_timer()
        self._gpsw_range.stop_timer()

    def _cancel(self) -> None:
        calib_cancel_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/rcin_calibration/cancel", Trigger)
        try:
            calib_cancel_sc.wait_for_service(self.WAIT_FOR_SERVER)
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

        self._reset()

    def _rcin_cb(self, msg: RCInput) -> None:
        if msg.error.error != RCInputError.E_NO_ERROR:
            q_error(self, "RC signal is not received.")
            self._cancel()
            return

        self._roll_range.set_value(msg.data[RCIN_ROLL])
        self._pitch_range.set_value(msg.data[RCIN_PITCH])
        self._yaw_range.set_value(msg.data[RCIN_YAW])
        self._thrust_range.set_value(msg.data[RCIN_THRUST])
        self._mode_range.set_value(msg.data[RCIN_MODE])
        self._estop_range.set_value(msg.data[RCIN_ESTOP])
        self._gpsw_range.set_value(msg.data[RCIN_GPSW])

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        calib_start_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/rcin_calibration/start", Trigger)
        try:
            calib_start_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return

        try:
            res: TriggerResponse = calib_start_sc.call(TriggerRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if not res.success:
            q_error(self, res.message)
            return

        # RC入力が正常に発行されていることを確認
        rcin_topic = f"/{self._drone.drone_name}/rcin_calibration/rc_input_raw"
        try:
            rcin_msg: RCInput = rospy.wait_for_message(rcin_topic, RCInput, self.WAIT_FOR_SERVER)
        except Exception:
            q_error(self, f"Failed to get RC input message in {self.WAIT_FOR_SERVER} seconds.")
            self._cancel()
            return
        if rcin_msg.error.error != RCInputError.E_NO_ERROR:
            q_error(self, "RC signal is not received.")
            self._cancel()
            return

        # 購読するトピックを更新
        self._rcin_sub = rospy.Subscriber(rcin_topic, RCInput, self._rcin_cb, queue_size=1)

        self._start_button.setEnabled(False)
        self._finish_button.setEnabled(True)
        self._cancel_button.setEnabled(True)

        self._start_timers()

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
        self._cancel()
        q_info(self._main, "Radio calibration is cancelled.")

    def _finish_calibration(self) -> bool:
        calib_finish_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/rcin_calibration/finish", RCInputCalibration)
        try:
            calib_finish_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = RCInputCalibrationRequest()
        req.roll_left = int(self._roll_range.get_lower())
        req.roll_right = int(self._roll_range.get_upper())
        req.pitch_down = int(self._pitch_range.get_upper())
        req.pitch_up = int(self._pitch_range.get_lower())
        req.yaw_left = int(self._yaw_range.get_lower())
        req.yaw_right = int(self._yaw_range.get_upper())
        req.thrust_down = int(self._thrust_range.get_upper())
        req.thrust_up = int(self._thrust_range.get_lower())
        req.mode_program = int(self._mode_range.get_lower())
        req.mode_stabilize = int(self._mode_range.get_middle())
        req.mode_acrobat = int(self._mode_range.get_upper())
        req.estop_on = int(self._estop_range.get_lower())
        req.estop_off = int(self._estop_range.get_upper())
        req.gpsw_on = int(self._gpsw_range.get_lower())
        req.gpsw_off = int(self._gpsw_range.get_upper())

        try:
            res: RCInputCalibrationResponse = calib_finish_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True

    def _reload_config(self) -> bool:
        reload_config_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/rcin_handler/reload_config", Trigger)
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
