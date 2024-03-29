from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget

from ...common import *


class BaseControlSystemSectionWidget(Widget):
    LABEL = UNKNOWN

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel(self.LABEL)
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        self._rows.addWidget(label)

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()
