from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget
from tobas_setup_assistant.setup_assistant import SetupAssistant


class SetupAssistantWidget(Widget):
    NAME = "Setup Assistant"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        self.setup_assistant = SetupAssistant(self)
        rows.addWidget(self.setup_assistant)

    def define_connections(self) -> None:
        pass
