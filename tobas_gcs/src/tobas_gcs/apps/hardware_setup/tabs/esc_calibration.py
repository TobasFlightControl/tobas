from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import BaseHardwareSetupWidget


class EscCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "ESC Calibration"
    TITLE = "Calibrate ESCs"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

    def define_connections(self) -> None:
        pass  # TODO
