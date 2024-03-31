from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..gcs import GroundControlStationWidget

from abc import abstractmethod

from tobas_rqt_tools.widgets import Widget
from tobas_tools_py.drone import Drone


class BaseAppWidget(Widget):
    NAME = "Undefined"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(parent=main)
        self._main = main
        self._drone = drone

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()
