from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget


class MissionPlannerWidget(BaseAppWidget):
    NAME = "Mission Planner"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

    @override
    def define_connections(self) -> None:
        pass  # TODO

    @override
    def update_internal_data_structures(self) -> None:
        pass  # TODO
