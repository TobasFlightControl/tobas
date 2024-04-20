from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget

from ...common import *
from ...parameter_getters import *


class BaseObserver(Widget):
    NAME = TO_DO
    PACKAGE_NAME = TO_DO

    def __init__(self, main: SetupAssistant, abst_text: str) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(abst_text)
        self._rows.addWidget(abst)

    @override
    def close(self) -> bool:
        return super().close()

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def static_parameters(self) -> dict:
        """静的ROSパラメータをまとめた辞書を返す．"""
        raise NotImplementedError()
