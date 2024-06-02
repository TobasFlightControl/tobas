from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from .base import BaseObserver


class CustomObserver(BaseObserver):
    NAME = "Use Custom Observer"
    PACKAGE_NAME = "tobas_dummy_pkg"
    ABST_TEXT = ""  # TODO: APIの案内など

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

    @override
    def static_parameters(self) -> dict:
        return dict()
