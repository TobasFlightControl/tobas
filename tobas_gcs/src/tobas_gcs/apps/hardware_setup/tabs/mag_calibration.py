from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import add_spacer

from ....common import Description
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

        add_spacer(self._rows)

    @override
    def define_connections(self) -> None:
        pass  # TODO
