from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from std_msgs.msg import Bool
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import LEDColor, LampWidget
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Gps, RCInput, RCInputError, PreArmCheck

from ....common import *
from .base_section import BaseControlSystemSectionWidget


class StatusWidget(QWidget):
    LED_SIZE = 20
    TEXT_PSIZE = 12

    def __init__(self, text: str) -> None:
        super().__init__()

        cols = QHBoxLayout()
        self.setLayout(cols)

        self._led = LampWidget()
        self._led.setFixedSize(self.LED_SIZE, self.LED_SIZE)
        self._led.set_color(LEDColor.BLACK)
        cols.addWidget(self._led)

        text_label = QLabel(text)
        text_label.setFont(QFont("Default", self.TEXT_PSIZE))
        cols.addWidget(text_label)

        cols.addStretch()

    def set_yes(self) -> None:
        self._led.set_color(LEDColor.GREEN)

    def set_no(self) -> None:
        self._led.set_color(LEDColor.RED)

    def set_unknown(self) -> None:
        self._led.set_color(LEDColor.BLACK)


class StatusViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Status"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._gps_status = StatusWidget("GPS Fix")
        self._rcin_status = StatusWidget("Radio Input")
        self._voltage_status = StatusWidget("Battery Voltage")
        self._attitude_status = StatusWidget("Attitude Stable")
        self._pos_accuracy_status = StatusWidget("Position Accuracy")
        self._rot_accuracy_status = StatusWidget("Orientation Accuracy")
        self._vel_accuracy_status = StatusWidget("Velocity Accuracy")
        self._ready_status = StatusWidget("Ready to Arm")
        self._arming_status = StatusWidget("Rotors Armed")

        grid = QGridLayout()
        self._rows.addLayout(grid)
        grid.addWidget(self._gps_status, 0, 0)
        grid.addWidget(self._rcin_status, 0, 1)
        grid.addWidget(self._voltage_status, 0, 2)
        grid.addWidget(self._attitude_status, 1, 0)
        grid.addWidget(self._pos_accuracy_status, 1, 1)
        grid.addWidget(self._rot_accuracy_status, 1, 2)
        grid.addWidget(self._vel_accuracy_status, 2, 0)
        grid.addWidget(self._ready_status, 2, 1)
        grid.addWidget(self._arming_status, 2, 2)
        grid.setColumnStretch(3, 1)

        self._gps_sub = None
        self._rcin_sub = None
        self._pre_arm_check_sub = None
        self._arming_sub = None

        self._is_first_update = True

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self._gps_status.set_unknown()
        self._rcin_status.set_unknown()
        self._voltage_status.set_unknown()
        self._attitude_status.set_unknown()
        self._pos_accuracy_status.set_unknown()
        self._rot_accuracy_status.set_unknown()
        self._vel_accuracy_status.set_unknown()
        self._ready_status.set_unknown()
        self._arming_status.set_unknown()

        if not self._is_first_update:
            self._gps_sub.unregister()
            self._rcin_sub.unregister()
            self._pre_arm_check_sub.unregister()
            self._arming_sub.unregister()

        self._gps_sub = rospy.Subscriber(f"{self._drone.drone_name}/gps", Gps, self._gps_cb, queue_size=1)
        self._rcin_sub = rospy.Subscriber(f"{self._drone.drone_name}/rc_input", RCInput, self._rcin_cb, queue_size=1)
        self._pre_arm_check_sub = rospy.Subscriber(
            f"{self._drone.drone_name}/pre_arm_check", PreArmCheck, self._pre_arm_check_cb, queue_size=1
        )
        self._arming_sub = rospy.Subscriber(f"{self._drone.drone_name}/arming", Bool, self._arming_cb, queue_size=1)

        self._is_first_update = False

    def _gps_cb(self, gps: Gps) -> None:
        if gps.fix_type == Gps.FIX_3D:
            self._gps_status.set_yes()
        else:
            self._gps_status.set_no()

    def _rcin_cb(self, rcin: RCInput) -> None:
        if rcin.error.error == RCInputError.E_NO_ERROR:
            self._rcin_status.set_yes()
        else:
            self._rcin_status.set_no()

    def _pre_arm_check_cb(self, msg: PreArmCheck) -> None:
        if msg.battery_voltage_ok:
            self._voltage_status.set_yes()
        else:
            self._voltage_status.set_no()

        if msg.attitude_ok:
            self._attitude_status.set_yes()
        else:
            self._attitude_status.set_no()

        if msg.position_accuracy_ok:
            self._pos_accuracy_status.set_yes()
        else:
            self._pos_accuracy_status.set_no()

        if msg.orientation_accuracy_ok:
            self._rot_accuracy_status.set_yes()
        else:
            self._rot_accuracy_status.set_no()

        if msg.velocity_accuracy_ok:
            self._vel_accuracy_status.set_yes()
        else:
            self._vel_accuracy_status.set_no()

        if msg.ok:
            self._ready_status.set_yes()
        else:
            self._ready_status.set_no()

    def _arming_cb(self, arming: Bool) -> None:
        if arming.data:
            self._arming_status.set_yes()
        else:
            self._arming_status.set_no()
