from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from ...common import TO_DO, TITLE_PSIZE


class BaseFixedWingSettingWidget(QWidget):
    NAME = TO_DO

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel(self.NAME)
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Weight.Bold))
        self._rows.addWidget(label)

    @abstractmethod
    def update_internal_data_structures(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def dump_settings(self) -> dict:
        raise NotImplementedError()

    @abstractmethod
    def load_settings(self, data: dict) -> None:
        raise NotImplementedError()
