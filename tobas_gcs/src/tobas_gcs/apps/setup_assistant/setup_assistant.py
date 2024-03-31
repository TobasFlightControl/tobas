from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.drone import Drone
from tobas_setup_assistant.setup_assistant import SetupAssistant

from ..base import BaseAppWidget


class SetupAssistantWidget(BaseAppWidget):
    NAME = "Setup Assistant"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self.setup_assistant = SetupAssistant(self)
        rows.addWidget(self.setup_assistant)

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        pass
