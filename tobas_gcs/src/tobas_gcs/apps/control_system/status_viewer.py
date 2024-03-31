from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import rospy
from abc import abstractmethod
from typing import final
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import LEDColor, LampWidget
from tobas_tools_py.drone import Drone

from .base_section import BaseControlSystemSectionWidget


class StatusViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Status"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._gps_status = GpsStatus(main, drone)
        self._rows.addWidget(self._gps_status)

    @override
    def define_connections(self) -> None:
        pass


class BaseStatusWidget(QWidget):
    TEXT = "Not defined"

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

    @final
    def set_on(self) -> None:
        self._led.set_color(LEDColor.GREEN)

    @final
    def set_off(self) -> None:
        self._led.set_color(LEDColor.RED)


class GpsStatus(BaseStatusWidget):
    TEXT = "GPS fix"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._gps_sub = None

    @override
    def define_connections(self) -> None:
        pass
