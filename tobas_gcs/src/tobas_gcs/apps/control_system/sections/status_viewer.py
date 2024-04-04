from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from std_msgs.msg import Bool
from abc import abstractmethod
from typing import final
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import LEDColor, LampWidget
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Gps, RCInput, RCInputError, Odometry, PreArmCheck

from ....common import *
from .base_section import BaseControlSystemSectionWidget


class StatusViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Status"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._gps_status = GpsStatus(main, drone)
        self._rcin_status = RCInputStatus(main, drone)
        self._odom_status = StateEstimationStatus(main, drone)
        self._pre_arm_check_status = PreArmCheckStatus(main, drone)
        self._arming_status = ArmingStatus(main, drone)

        self._rows.addWidget(self._gps_status)
        self._rows.addWidget(self._rcin_status)
        self._rows.addWidget(self._odom_status)
        self._rows.addWidget(self._pre_arm_check_status)
        self._rows.addWidget(self._arming_status)

    @override
    def define_connections(self) -> None:
        self._gps_status.define_connections()
        self._rcin_status.define_connections()
        self._odom_status.define_connections()
        self._pre_arm_check_status.define_connections()
        self._arming_status.define_connections()

    @override
    def update_internal_data_structures(self) -> None:
        self._gps_status.update_internal_data_structures()
        self._rcin_status.update_internal_data_structures()
        self._odom_status.update_internal_data_structures()
        self._pre_arm_check_status.update_internal_data_structures()
        self._arming_status.update_internal_data_structures()


class BaseStatusWidget(QWidget):
    TEXT = TO_DO

    LED_SIZE = 20
    TEXT_PSIZE = 12

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        cols = QHBoxLayout()
        self.setLayout(cols)

        self._led = LampWidget()
        self._led.setFixedSize(self.LED_SIZE, self.LED_SIZE)
        self._led.set_color(LEDColor.BLACK)
        cols.addWidget(self._led)

        text = QLabel(self.TEXT)
        text.setFont(QFont("Default", self.TEXT_PSIZE))
        cols.addWidget(text)

        cols.addStretch()

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def update_internal_data_structures(self) -> None:
        raise NotImplementedError()

    @final
    def set_yes(self) -> None:
        self._led.set_color(LEDColor.GREEN)

    @final
    def set_no(self) -> None:
        self._led.set_color(LEDColor.RED)

    @final
    def set_unknown(self) -> None:
        self._led.set_color(LEDColor.BLACK)


class GpsStatus(BaseStatusWidget):
    TEXT = "GPS Fix"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._gps_sub = None

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self.set_unknown()

        if self._gps_sub is not None:
            self._gps_sub.unregister()
        self._gps_sub = rospy.Subscriber(f"/{self._drone.drone_name}/gps", Gps, self._gps_cb, queue_size=1)

    def _gps_cb(self, gps: Gps) -> None:
        if gps.fix_type == Gps.FIX_3D:
            self.set_yes()
        else:
            self.set_no()


class RCInputStatus(BaseStatusWidget):
    TEXT = "Radio Input"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._rcin_sub = None

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self.set_unknown()

        if self._rcin_sub is not None:
            self._rcin_sub.unregister()
        self._rcin_sub = rospy.Subscriber(f"/{self._drone.drone_name}/rc_input", RCInput, self._rcin_cb, queue_size=1)

    def _rcin_cb(self, rcin: RCInput) -> None:
        if rcin.error.error == RCInputError.E_NO_ERROR:
            self.set_yes()
        else:
            self.set_no()


class StateEstimationStatus(BaseStatusWidget):
    TEXT = "State Estimation"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._odom_sub = None

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self.set_unknown()

        if self._odom_sub is not None:
            self._odom_sub.unregister()
        self._odom_sub = rospy.Subscriber(f"/{self._drone.drone_name}/odom", Odometry, self._odom_cb, queue_size=1)

    def _odom_cb(self, odom: Odometry) -> None:
        if odom.status == Odometry.NO_ERROR:
            self.set_yes()
        else:
            self.set_no()


class PreArmCheckStatus(BaseStatusWidget):
    TEXT = "Pre-Arm Check"

    PRE_ARM_CHECK_PERIOD = 1000  # [ms]

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._pre_arm_check_sub = None

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self.set_unknown()

        if self._pre_arm_check_sub is not None:
            self._pre_arm_check_sub.unregister()
        self._pre_arm_check_sub = rospy.Subscriber(
            f"/{self._drone.drone_name}/pre_arm_check", PreArmCheck, self._pre_arm_check_cb, queue_size=1
        )

    def _pre_arm_check_cb(self, msg: PreArmCheck) -> None:
        if msg.error_code >= 0:
            self.set_yes()
        else:
            self.set_no()


class ArmingStatus(BaseStatusWidget):
    TEXT = "Rotors Armed"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._arming_sub = None

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self.set_unknown()

        if self._arming_sub is not None:
            self._arming_sub.unregister()
        self._arming_sub = rospy.Subscriber(f"/{self._drone.drone_name}/arming", Bool, self._arming_cb, queue_size=1)

    def _arming_cb(self, arming: Bool) -> None:
        if arming.data:
            self.set_yes()
        else:
            self.set_no()
