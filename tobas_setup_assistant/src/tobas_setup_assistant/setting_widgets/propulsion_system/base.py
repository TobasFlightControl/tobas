from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtWidgets import QWidget

from ...common import TO_DO


class BaseSelectedLinkSettingWidget(QWidget):
    NAME = TO_DO

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def copy_from(self, src: BaseSelectedLinkSettingWidget) -> None:
        raise NotImplementedError()

    @abstractmethod
    def dump_settings(self) -> dict:
        raise NotImplementedError()

    @abstractmethod
    def load_settings(self, data: dict) -> None:
        raise NotImplementedError()
