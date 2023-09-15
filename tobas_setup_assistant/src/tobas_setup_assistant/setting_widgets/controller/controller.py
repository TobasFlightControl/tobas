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
from .base import BaseController
from .multirotor_lmpc import MultirotorLMPC
from .fixed_wing_lqr import FixedWingLQR


class ControllerWidget(BaseSettingWidget):
    NAME = "Controller"

    NO_SELECT = "Select controller type"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Controller"
        abst_text = (
            "飛行制御器の設定を行います．"
            + "手法を1つ選択し，各パラメータを設定してください．"
            + "パラメータは後からチューニングすることもできるので，デフォルトのままでも構いません．"
        )
        super().__init__(main, title_text, abst_text)

        self.type = ComboBox()
        self.type.addItem(self.NO_SELECT)
        self._rows.addWidget(self.type)

        self.multirotor_lmpc = MultirotorLMPC(main)
        self._rows.addWidget(self.multirotor_lmpc)

        self.fixed_wing_lqr = FixedWingLQR(main)
        self._rows.addWidget(self.fixed_wing_lqr)

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self.type.currentTextChanged.connect(self._on_type_changed)
        self._main.signals.airframe_updated.connect(self._on_airframe_updated)
        
        self.multirotor_lmpc.define_connections()
        self.fixed_wing_lqr.define_connections()

    @overrides
    def is_valid(self) -> bool:
        if self.type.currentText() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select controller type.")
            return False

        if not self.selected().is_valid():
            return False

        return True

    def selected(self) -> BaseController:
        controller_type = self.type.currentText()

        if controller_type == self.NO_SELECT:
            raise RuntimeError("Controller type is not selected.")
        if controller_type == MultirotorLMPC.NAME:
            return self.multirotor_lmpc
        elif controller_type == FixedWingLQR.NAME:
            return self.fixed_wing_lqr
        else:
            raise RuntimeError(f"Invalid controller type: {controller_type}")

    def get_type(self) -> str:
        return self.type.currentText()

    def controller_pkg(self) -> str:
        return self.selected().CONTROLLER_PKG

    def takeoff_pkg(self) -> str:
        return self.selected().TAKEOFF_PKG

    def landing_pkg(self) -> str:
        return self.selected().LANDING_PKG

    def _update_controller_types(self) -> None:
        if self.multirotor_lmpc.is_applicable():
            if not self.type.contains(MultirotorLMPC.NAME):
                self.type.addItem(MultirotorLMPC.NAME)
        else:
            if self.type.contains(MultirotorLMPC.NAME):
                self.type.setCurrentText(self.NO_SELECT)
                self.type.remove_text(MultirotorLMPC.NAME)

        if self.fixed_wing_lqr.is_applicable():
            if not self.type.contains(FixedWingLQR.NAME):
                self.type.addItem(FixedWingLQR.NAME)
        else:
            if self.type.contains(FixedWingLQR.NAME):
                self.type.setCurrentText(self.NO_SELECT)
                self.type.remove_text(FixedWingLQR.NAME)

        # 選択可能なコントローラが1種類の場合は自動的にそれを選択
        if self.type.count() == 2:
            self.type.setCurrentIndex(1)  # idx = 0がNO_SELECTで，その次に設定

    def _update_visibility(self) -> None:
        controller_type = self.type.currentText()

        if controller_type == self.NO_SELECT:
            self.multirotor_lmpc.setVisible(False)
            self.fixed_wing_lqr.setVisible(False)
        elif controller_type == MultirotorLMPC.NAME:
            self.multirotor_lmpc.setVisible(True)
            self.fixed_wing_lqr.setVisible(False)
        elif controller_type == FixedWingLQR.NAME:
            self.multirotor_lmpc.setVisible(False)
            self.fixed_wing_lqr.setVisible(True)
        else:
            raise RuntimeError(f"Unknown controller type: {controller_type}")

    @pyqtSlot(str)
    def _on_type_changed(self, controller_type: str) -> None:
        self._update_visibility()

    @pyqtSlot()
    def _on_airframe_updated(self) -> None:
        self._update_controller_types()
        self._update_visibility()
