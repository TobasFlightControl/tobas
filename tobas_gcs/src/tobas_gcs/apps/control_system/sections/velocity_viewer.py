from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from overrides import override

from tobas_tools_py.drone import Drone

from .base_section import BaseControlSystemSectionWidget


class VelocityViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Velocity"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        # TODO

    @override
    def update_internal_data_structures(self) -> None:
        pass
