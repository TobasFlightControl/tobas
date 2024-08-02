from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import override

from .base import BaseHardwareWidget


class Navio2Widget(BaseHardwareWidget):
    NAME = "Navio2 | Emlid"
    PACKAGE_NAME = "tobas_navio_ros"
    ABST_TEXT = ""  # TODO

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def dump_settings(self) -> dict:
        return dict()

    @override
    def load_settings(self, data: dict) -> None:
        pass
