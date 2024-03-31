from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from abc import abstractmethod
from typing import final
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import LEDColor, LampWidget
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Gps, RCInput, RCInputError, Odometry

from ....common import TO_DO
from .base_section import BaseControlSystemSectionWidget


class StatusViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Status"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._gps_status = GpsStatus(main, drone)
        self._rcin_status = RCInputStatus(main, drone)
        self._ready_status = ReadyToFlightStatus(main, drone)

        self._rows.addWidget(self._gps_status)
        self._rows.addWidget(self._rcin_status)
        self._rows.addWidget(self._ready_status)

    @override
    def define_connections(self) -> None:
        self._gps_status.define_connections()
        self._rcin_status.define_connections()
        self._ready_status.define_connections()

    @override
    def update_internal_data_structures(self) -> None:
        self._gps_status.update_internal_data_structures()
        self._rcin_status.update_internal_data_structures()
        self._ready_status.update_internal_data_structures()


class BaseStatusWidget(QWidget):
    TEXT = TO_DO

    LED_SIZE = 20
    TEXT_PSIZE = 12

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(parent=main)
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
    TEXT = "GPS fix"

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
    TEXT = "Radio input received"

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
        self._rcin_sub = rospy.Subscriber(f"/{self._drone.drone_name}/rc_input", Gps, self._rcin_cb, queue_size=1)

    def _rcin_cb(self, rcin: RCInput) -> None:
        if rcin.error == RCInputError.E_NO_ERROR:
            self.set_yes()
        else:
            self.set_no()


class ReadyToFlightStatus(BaseStatusWidget):
    TEXT = "Ready to flight"

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
        self._odom_sub = rospy.Subscriber(f"/{self._drone.drone_name}/odom", Gps, self._odom_cb, queue_size=1)

    def _odom_cb(self, _: Odometry) -> None:
        # TODO: 状態推定だけでなく，アクチュエータのチェックも行う
        self.set_yes()
