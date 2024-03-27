from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget


class ControlSystemWidget(Widget):
    NAME = "Control System"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

    def define_connections(self) -> None:
        pass  # TODO
