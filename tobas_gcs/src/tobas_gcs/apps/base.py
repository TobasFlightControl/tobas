from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..gcs import GroundControlStationWidget

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..common import *


class BaseAppWidget(QWidget):
    NAME = UNKNOWN

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
        self._main = main

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()
