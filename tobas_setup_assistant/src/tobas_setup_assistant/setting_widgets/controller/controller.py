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
from .base import BaseController
from .multirotor_pid import MultirotorPid
from .multirotor_mpc import MultirotorMpc
from .fixed_wing_lqr import FixedWingLQR
from .arducopter import ArduCopter


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

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._rows.addWidget(self._type)

        self._controllers: List[BaseController] = [
            MultirotorPid(main),
            MultirotorMpc(main),
            ArduCopter(main),
            FixedWingLQR(main),
        ]
        for controller in self._controllers:
            self._rows.addWidget(controller)

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._type.currentTextChanged.connect(self._on_type_changed)
        self._main.signals.airframe_updated.connect(self._on_airframe_updated)

        for controller in self._controllers:
            controller.define_connections()

    @overrides
    def is_valid(self) -> bool:
        if self._type.currentText() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select controller type.")
            return False

        if not self.selected().is_valid():
            return False

        return True

    def selected(self) -> BaseController:
        controller_type = self._type.currentText()

        if controller_type == self.NO_SELECT:
            raise RuntimeError("Controller type is not selected.")

        for controller in self._controllers:
            if controller_type == controller.NAME:
                return controller

        raise RuntimeError(f"Invalid controller type: {controller_type}")

    def get_type(self) -> str:
        return self._type.currentText()

    def controller_pkg(self) -> str:
        return self.selected().CONTROLLER_PKG

    def takeoff_pkg(self) -> str:
        return self.selected().TAKEOFF_PKG

    def landing_pkg(self) -> str:
        return self.selected().LANDING_PKG

    def _update_controller_types(self) -> None:
        for controller in self._controllers:
            name = controller.NAME
            if controller.is_applicable():
                if not self._type.contains(name):
                    self._type.addItem(name)
            else:
                if self._type.contains(name):
                    self._type.setCurrentText(self.NO_SELECT)
                    self._type.remove_text(name)

        # 選択可能なコントローラが1種類の場合は自動的にそれを選択
        if self._type.count() == 2:
            self._type.setCurrentIndex(1)  # idx = 0がNO_SELECTで，その次に設定

    def _update_visibility(self) -> None:
        controller_type = self._type.currentText()

        for controller in self._controllers:
            controller.setVisible(False)

        for controller in self._controllers:
            if controller.NAME == controller_type:
                controller.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, controller_type: str) -> None:
        self._update_visibility()

    @pyqtSlot()
    def _on_airframe_updated(self) -> None:
        self._update_controller_types()
        self._update_visibility()
