from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import List
from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from ..base_setting import BaseSettingWidget
from .base import BaseObserver
from .cascade import CascadeKalmanFilter
from .eskf import ErrorStateKalmanFilter


class ObserverWidget(BaseSettingWidget):
    NAME = "Observer"

    NO_SELECT = "Select observer type"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Observer"
        abst_text = (
            "状態推定器の設定を行います．"
            + "手法を1つ選択し，各パラメータを設定してください．"
            + "パラメータは後からチューニングすることもできるので，デフォルトのままでも構いません．"
        )
        super().__init__(main, title_text, abst_text)

        self._observers: List[BaseObserver] = [
            CascadeKalmanFilter(main),
            ErrorStateKalmanFilter(main),
        ]

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._rows.addWidget(self._type)

        for observer in self._observers:
            self._rows.addWidget(observer)
            self._type.addItem(observer.NAME)

        self._type.setCurrentText(ErrorStateKalmanFilter.NAME)  # Default

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._type.currentTextChanged.connect(self._on_type_changed)

    @overrides
    def is_valid(self) -> bool:
        if self._type.currentText() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select observer type.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    def pkg_name(self) -> str:
        return self._selected().PACKAGE_NAME

    def parameter_dict(self) -> dict:
        return self._selected().parameter_dict()

    def _selected(self) -> BaseObserver:
        observer_type = self._type.currentText()

        if observer_type == self.NO_SELECT:
            raise RuntimeError("Observer type is not selected.")

        for observer in self._observers:
            if observer_type == observer.NAME:
                return observer

        RuntimeError(f"Unknown observer type: {observer_type}")

    def _update_visibility(self) -> None:
        observer_type = self._type.currentText()

        for observer in self._observers:
            observer.setVisible(False)

        for observer in self._observers:
            if observer.NAME == observer_type:
                observer.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, _: str) -> None:
        self._update_visibility()
