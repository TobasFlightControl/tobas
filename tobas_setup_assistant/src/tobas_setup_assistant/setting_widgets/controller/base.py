from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from tobas_rqt_tools.widgets import Widget

from ...common import TO_DO, Description


class BaseController(Widget):
    NAME = TO_DO
    CONTROLLER_PKG = TO_DO
    TAKEOFF_PKG = TO_DO
    LANDING_PKG = TO_DO
    MOVE_PKG = TO_DO
    STABLIZE_MODE = TO_DO
    ACROBAT_MODE = TO_DO

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
    def define_connections(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def is_applicable(self) -> bool:
        """
        ハードウェアの構造のみから，制御器が適用可能かどうかを返す．

        Returns
        -------
        bool
            制御器が適用可能かどうか．

        Note
        ------
        - 実験データによるモータの設定など，個別の設定方法に依存してはならない．
        """
        raise NotImplementedError()

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def static_parameters(self) -> dict:
        """静的プライベートROSパラメータをまとめた辞書を返す．"""
        raise NotImplementedError()
