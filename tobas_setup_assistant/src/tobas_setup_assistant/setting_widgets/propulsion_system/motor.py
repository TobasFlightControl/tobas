from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget
    from .motor_dynamics import MotorDynamicsWidget_Base

from overrides import override
from typing import Tuple, List
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ...common import CW, CCW
from ...parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_ComboBox
from .common import PROPULSION_SYSTEM
from .base import BaseSelectedLinkSettingWidget
from .max_rot_speed import MaxRotationSpeedWidget
from .motor_dynamics import MotorDynamicsWidget_Spec, MotorDynamicsWidget_Experiment


class MotorWidget(BaseSelectedLinkSettingWidget):
    NAME = "Motor Settings"

    NO_SELECT = "Select setting method"

    COMMON_PARAMS_KEY = "common_params"
    METHOD_NAME_KEY = "method_name"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        self._common_params = MotorCommonParamsWidget(main, link_name)
        self._rows.addWidget(self._common_params)

        self._methods: List[MotorDynamicsWidget_Base] = [
            MotorDynamicsWidget_Spec(main, link_name),
            MotorDynamicsWidget_Experiment(main, link_name),
        ]

        self._method_name = ComboBox()
        self._method_name.currentTextChanged.connect(self._on_type_changed)
        self._rows.addWidget(self._method_name)

        self._method_name.addItem(self.NO_SELECT)
        for method in self._methods:
            self._method_name.addItem(method.NAME)
            self._rows.addWidget(method)

        self._update_visibility()

    @override
    def is_valid(self) -> bool:
        if not self._common_params.is_valid():
            return False

        if self._method_name.currentText() == self.NO_SELECT:
            q_error_named(self._main, PROPULSION_SYSTEM, "Please select motor setting method.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    @override
    def copy_from(self, src: MotorWidget) -> None:
        self._common_params.copy_from(src._common_params)
        self._method_name.setCurrentText(src._method_name.currentText())
        for des_method, src_method in zip(self._methods, src._methods):
            des_method.copy_from(src_method)

    @override
    def dump_settings(self) -> dict:
        res = dict()

        res[self.COMMON_PARAMS_KEY] = self._common_params.dump_settings()
        res[self.METHOD_NAME_KEY] = self._method_name.currentText()
        for method in self._methods:
            res[method.NAME] = method.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._common_params.load_settings(data[self.COMMON_PARAMS_KEY])
        self._method_name.setCurrentText(data[self.METHOD_NAME_KEY])
        for method in self._methods:
            method.load_settings(data[method.NAME])

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        return self._common_params.max_rot_speed.max_rot_speed()

    def direction(self) -> str:
        """CW or CCW"""
        return self._common_params.direction.get()

    def num_poles(self) -> float:
        return self._common_params.num_poles.get()

    def time_const_up(self) -> float:
        """[s]"""
        return self._common_params.time_const_up.get() * 1e-3

    def time_const_down(self) -> float:
        """[s]"""
        return self._common_params.time_const_down.get() * 1e-3

    def rot_speed_coefs(self) -> Tuple[float, float]:
        """V = a w + b w^2 (V[V], w[rad/s])"""
        return self._selected().rot_speed_coefs()

    def _selected(self) -> MotorDynamicsWidget_Base:
        method_name = self._method_name.currentText()

        if method_name == self.NO_SELECT:
            raise RuntimeError("Setting method is not selected.")

        for method in self._methods:
            if method_name == method.NAME:
                return method

        raise RuntimeError(f"Invalid setting method: {method_name}")

    def _update_visibility(self) -> None:
        method_name = self._method_name.currentText()

        for method in self._methods:
            method.setVisible(False)

        for method in self._methods:
            if method.NAME == method_name:
                method.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, _: str) -> None:
        self._update_visibility()


class MotorCommonParamsWidget(QWidget):
    MAX_ROT_SPEED_KEY = "max_rot_speed"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        # Max Rotation Speed
        self.max_rot_speed = MaxRotationSpeedWidget(main, link_name)
        self._rows.addWidget(self.max_rot_speed)

        # Param Getter Widgets
        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        direction_description = (
            "Motor rotation direction. "
            "Please choose either CW (Clockwise) or CCW (Counter Clockwise) relative to the rotation axis. "
            "For instance, in rotary-wing aircraft, "
            "propellers positioned diagonally opposite each other typically rotate in the same direction."
        )
        self.direction = ParamGetterWidget_ComboBox("Rotating Direction", direction_description, [CW, CCW])
        self._param_rows.addWidget(self.direction)

        num_poles_description = ""  # TODO
        self.num_poles = ParamGetterWidget_SpinBox("The number of poles", num_poles_description, minimum=2, default=14)
        self._param_rows.addWidget(self.num_poles)

        time_const_up_description = (
            "Time constant of the motor's response when increasing its rotational speed, "
            "relative to the command value."
        )
        self.time_const_up = ParamGetterWidget_SpinBox(
            "Time Constant Up", time_const_up_description, minimum=1, default=15, suffix=" ms"
        )
        self._param_rows.addWidget(self.time_const_up)

        time_const_down_description = (
            "Time constant of the motor's response when decreasing its rotational speed, "
            "relative to the command value."
        )
        self.time_const_down = ParamGetterWidget_SpinBox(
            "Time Constant Down", time_const_down_description, minimum=1, default=30, suffix=" ms"
        )
        self._param_rows.addWidget(self.time_const_down)

    def is_valid(self) -> bool:
        if not self.max_rot_speed.is_valid():
            return False

        if self.num_poles.get() % 2 == 1:
            q_error_named(self, PROPULSION_SYSTEM, "The number of poles of a brushless motor must be even.")
            return False

        return True

    def copy_from(self, src: MotorCommonParamsWidget) -> None:
        self.max_rot_speed.copy_from(src.max_rot_speed)

        for i in range(self._param_rows.count()):
            param_des: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param_src: ParamGetterWidget = src._param_rows.itemAt(i).widget()
            param_des.set(param_src.get())

    def dump_settings(self) -> dict:
        res = dict()

        res[self.MAX_ROT_SPEED_KEY] = self.max_rot_speed.dump_settings()

        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()

        return res

    def load_settings(self, data: dict) -> None:
        self.max_rot_speed.load_settings(data[self.MAX_ROT_SPEED_KEY])

        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])
