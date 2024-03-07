from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import ScrollArea

from ....common import *


class BaseHardwareSetupWidget(ScrollArea):
    ABST_HEIGHT = 100
    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100
    WAIT_FOR_SERVICE = 1  # [s]

    NAME = UNKNOWN
    TITLE = UNKNOWN

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
        self._main = main

        self.setWidgetResizable(True)  # この設定が必須．無いとオブジェクトが潰れてしまう．

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel(self.TITLE)
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        spacer = QSpacerItem(0, 50)
        self._rows.addItem(spacer)

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()
