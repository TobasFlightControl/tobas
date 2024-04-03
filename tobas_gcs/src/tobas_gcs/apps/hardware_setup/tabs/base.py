from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import ScrollArea
from tobas_tools_py.drone import Drone

from ....common import *


class BaseHardwareSetupWidget(ScrollArea):
    ABST_HEIGHT = 100
    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40
    WAIT_FOR_SERVER = 1  # [s]

    E_FAILED_TO_CONNECT = "Failed to connect to the flight controller."
    E_FAILED_TO_CALL_SRV = "Failed to call ROS service"

    NAME = TO_DO
    TITLE = TO_DO

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel(self.TITLE)
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self._rows.addSpacing(50)

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def update_internal_data_structures(self) -> None:
        raise NotImplementedError()
