from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from ...common import TO_DO, Description


class BaseHardwareWidget(QWidget):
    NAME = TO_DO
    PACKAGE_NAME = TO_DO
    ABST_TEXT = TO_DO

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(self.ABST_TEXT)
        self._rows.addWidget(abst)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def dump_settings(self) -> dict:
        raise NotImplementedError()

    @abstractmethod
    def load_settings(self, data: dict) -> None:
        raise NotImplementedError()
