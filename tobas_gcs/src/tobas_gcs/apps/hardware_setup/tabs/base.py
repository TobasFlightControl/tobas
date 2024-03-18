from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os.path as osp
from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import ScrollArea
from tobas_tools_py.drone import Drone, DroneLoader_File

from ....common import *


class BaseHardwareSetupWidget(ScrollArea):
    ABST_HEIGHT = 100
    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100
    WAIT_FOR_SERVER = 1  # [s]

    E_FAILED_TO_CONNECT = "Failed to connect to the flight controller."

    NAME = UNKNOWN
    TITLE = UNKNOWN

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()

        self._main = main
        self._drone = Drone()

        self.setWidgetResizable(True)  # この設定が必須．無いとオブジェクトが潰れてしまう．
        self.setEnabled(False)  # configパッケージが読み込まれたら有効化

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel(self.TITLE)
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self._rows.addSpacing(50)

    @abstractmethod
    def define_connections(self) -> None:
        self._main.signals.config_pkg_updated.connect(self._on_config_pkg_updated)

    @abstractmethod
    @pyqtSlot(str)
    def _on_config_pkg_updated(self, pkg_path: str) -> None:
        tbsf_path = osp.join(pkg_path, "config/drone.tbsf")
        DroneLoader_File(self._drone, tbsf_path).load()

        self.setEnabled(True)
