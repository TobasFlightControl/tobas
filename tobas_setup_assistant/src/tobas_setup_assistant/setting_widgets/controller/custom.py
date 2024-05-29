from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from .base import BaseController


class CustomController(BaseController):
    NAME = "Use Custom Controller"
    CONTROLLER_PKG = "tobas_dummy_pkg"
    TAKEOFF_PKG = "tobas_dummy_pkg"
    LANDING_PKG = "tobas_dummy_pkg"
    MOVE_PKG = "tobas_dummy_pkg"
    STABLIZE_MODE = ""
    ACROBAT_MODE = ""

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = ""  # TODO: APIの案内など
        super().__init__(main, abst_text)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_applicable(self) -> bool:
        return True

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def static_parameters(self) -> dict:
        return dict()
