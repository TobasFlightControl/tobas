from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from abc import abstractmethod
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import LEDColor, LampWidget

from .base_section import BaseControlSystemSectionWidget


class StatusViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Status"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        self._gps_status = GpsStatus(main)
        self._rows.addWidget(self._gps_status)

    @override
    def define_connections(self) -> None:
        pass


class BaseStatusWidget(QWidget):
    TEXT = "Not defined"

    LED_SIZE = 20
    TEXT_PSIZE = 12

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

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


class GpsStatus(BaseStatusWidget):
    TEXT = "GPS fix"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)
