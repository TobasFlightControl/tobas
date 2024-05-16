from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from .base import BaseObserver


class CustomObserver(BaseObserver):
    NAME = "Use Custom Observer"
    PACKAGE_NAME = "tobas_dummy_pkg"

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = ""  # TODO: APIの案内など
        super().__init__(main, abst_text)

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def static_parameters(self) -> dict:
        return dict()
