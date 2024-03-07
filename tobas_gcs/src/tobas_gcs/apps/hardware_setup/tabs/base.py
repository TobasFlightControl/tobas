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

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()
