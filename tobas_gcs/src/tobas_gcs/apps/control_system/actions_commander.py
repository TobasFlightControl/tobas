from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_section import BaseControlSystemSectionWidget


class ActionsCommanderWidget(BaseControlSystemSectionWidget):
    LABEL = "Actions"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        # TODO

    @override
    def define_connections(self) -> None:
        pass
