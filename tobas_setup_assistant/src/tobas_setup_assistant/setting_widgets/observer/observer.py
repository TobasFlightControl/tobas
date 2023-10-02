from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

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

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._type.addItem(CascadeKalmanFilter.NAME)
        self._type.addItem(ErrorStateKalmanFilter.NAME)
        self._type.setCurrentText(ErrorStateKalmanFilter.NAME)  # Default
        self._rows.addWidget(self._type)

        self._cascade = CascadeKalmanFilter(main)
        self._rows.addWidget(self._cascade)

        self._eskf = ErrorStateKalmanFilter(main)
        self._rows.addWidget(self._eskf)

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._type.currentTextChanged.connect(self._on_type_changed)

    @overrides
    def is_valid(self) -> bool:
        if self.get_type() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select observer type.")
            return False

        if not self.selected().is_valid():
            return False

        return True

    def selected(self) -> BaseObserver:
        observer_type = self.get_type()

        if observer_type == self.NO_SELECT:
            raise RuntimeError("Observer type is not selected.")
        elif observer_type == CascadeKalmanFilter.NAME:
            return self._cascade
        elif observer_type == ErrorStateKalmanFilter.NAME:
            return self._eskf
        else:
            raise RuntimeError(f"Unknown observer type: {observer_type}")

    def get_type(self) -> str:
        return self._type.currentText()

    def pkg_name(self) -> str:
        return self.selected().PACKAGE_NAME

    @pyqtSlot(str)
    def _on_type_changed(self, observer_type: str) -> None:
        self._update_visibility()

    def _update_visibility(self) -> None:
        observer_type = self.get_type()

        if observer_type == self.NO_SELECT:
            self._cascade.setVisible(False)
            self._eskf.setVisible(False)
        elif observer_type == CascadeKalmanFilter.NAME:
            self._cascade.setVisible(True)
            self._eskf.setVisible(False)
        elif observer_type == ErrorStateKalmanFilter.NAME:
            self._cascade.setVisible(False)
            self._eskf.setVisible(True)
        else:
            raise RuntimeError(f"Unknown observer type: {observer_type}")
