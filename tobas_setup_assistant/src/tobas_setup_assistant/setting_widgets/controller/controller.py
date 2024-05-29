from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import List
from overrides import override
from PyQt5.QtCore import pyqtSlot

from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ..base_setting import BaseSettingWidget
from .base import BaseController
from .multirotor_pid import MultirotorPid
from .multirotor_mpc import MultirotorMpc
from .arducopter import ArduCopter
from .non_planar_pid import NonPlanarPid
from .fixed_wing_lqr import FixedWingLQR
from .custom import CustomController


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
            CustomController(main),
        ]

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._type.currentTextChanged.connect(self._on_type_changed)
        self._rows.addWidget(self._type)

        for controller in self._controllers:
            self._rows.addWidget(controller)

        self._rows.addStretch()
        self._update_visibility()

        self._main.signals.airframe_updated.connect(self._on_airframe_updated)

    @override
    def update_internal_data_structures(self) -> None:
        for controller in self._controllers:
            controller.update_internal_data_structures()

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

    def move_pkg(self) -> str:
        return self._selected().MOVE_PKG

    def stabilize_mode(self) -> str:
        return self._selected().STABLIZE_MODE

    def acrobat_mode(self) -> str:
        return self._selected().ACROBAT_MODE

    def static_parameters(self) -> dict:
        return self._selected().static_parameters()

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
