from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import joblib
from typing import List
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from ..base_setting import BaseSettingWidget
from .base import BaseController
from .multirotor_pid import MultirotorPid
from .multirotor_mpc import MultirotorMpc
from .non_planar_pid import NonPlanarPid
from .fixed_wing_lqr import FixedWingLQR
from .arducopter import ArduCopter


class ControllerWidget(BaseSettingWidget):
    NAME = "Controller"

    NO_SELECT = "Select controller type"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Controller"
        abst_text = (
            "Configure the flight controller by selecting one method and setting its parameters. "
            "You can fine-tune the parameters later, "
            "so it's acceptable to leave them at their default settings initially."
        )
        super().__init__(main, title_text, abst_text)

        self._controllers: List[BaseController] = [
            MultirotorPid(main),
            MultirotorMpc(main),
            ArduCopter(main),
            NonPlanarPid(main),
            FixedWingLQR(main),
        ]

        # 各制御器の動的パラメータを並列に読み込む
        # NOTE: ウィジェット自体をマルチスレッドにすると親子関係が壊れるため，コンストラクタの並列処理はできない
        job = joblib.Parallel(n_jobs=-1, prefer="threads")  # メモリ共有するためマルチプロセスではなくマルチスレッド
        job(joblib.delayed(ctrl.get_dynamic_params)() for ctrl in self._controllers)

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._rows.addWidget(self._type)

        for controller in self._controllers:
            controller.add_dynamic_params()
            self._rows.addWidget(controller)

        self._rows.addStretch()
        self._update_visibility()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._type.currentTextChanged.connect(self._on_type_changed)
        self._main.signals.airframe_updated.connect(self._on_airframe_updated)

        for controller in self._controllers:
            controller.define_connections()

    @override
    def is_valid(self) -> bool:
        if self._type.currentText() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select controller type.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    def controller_pkg(self) -> str:
        return self._selected().CONTROLLER_PKG

    def takeoff_pkg(self) -> str:
        return self._selected().TAKEOFF_PKG

    def landing_pkg(self) -> str:
        return self._selected().LANDING_PKG

    def stabilize_mode(self) -> str:
        return self._selected().STABLIZE_MODE

    def acrobat_mode(self) -> str:
        return self._selected().ACROBAT_MODE

    def parameter_dict(self) -> dict:
        return self._selected().parameter_dict()

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
        else:
            self._type.setCurrentIndex(0)  # NO_SELECT

    def _selected(self) -> BaseController:
        controller_type = self._type.currentText()

        if controller_type == self.NO_SELECT:
            raise RuntimeError("Controller type is not selected.")

        for controller in self._controllers:
            if controller_type == controller.NAME:
                return controller

        raise RuntimeError(f"Invalid controller type: {controller_type}")

    def _update_visibility(self) -> None:
        controller_type = self._type.currentText()

        for controller in self._controllers:
            controller.setVisible(False)

        for controller in self._controllers:
            if controller.NAME == controller_type:
                controller.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, _: str) -> None:
        self._update_visibility()

    @pyqtSlot()
    def _on_airframe_updated(self) -> None:
        self._update_controller_types()
        self._update_visibility()
