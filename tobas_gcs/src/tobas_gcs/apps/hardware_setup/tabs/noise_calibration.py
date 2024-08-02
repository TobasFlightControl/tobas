from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from typing import override

from tobas_tools_py.drone import Drone

from .base import BaseHardwareSetupWidget


class NoiseCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Sensor Noise"
    TITLE = "Measure Sensor Noise"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self.setEnabled(False)

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)
