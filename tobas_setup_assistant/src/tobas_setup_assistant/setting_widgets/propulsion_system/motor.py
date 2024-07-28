from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from tobas_std_tools_py.math import rpm2rps
from tobas_rqt_tools.messages import q_error_named
from tobas_tools_py.rotor_config import TurningDirection

from ...parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_ComboBox
from .common import PROPULSION_SYSTEM
from .base import BaseSelectedLinkSettingWidget
from .max_rot_speed import MaxRotationSpeedWidget


class MotorWidget(BaseSelectedLinkSettingWidget):
    NAME = "Motor"

    MAX_ROT_SPEED_KEY = "max_rot_speed"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        rows = QVBoxLayout()
        self.setLayout(rows)

        # Param Getter Widgets
        self._param_rows = QVBoxLayout()
        rows.addLayout(self._param_rows)

        direction_description = (
            "Motor rotation direction. "
            "Please choose either CW (Clockwise) or CCW (Counter Clockwise) relative to the rotation axis. "
            "For instance, in rotary-wing aircraft, "
            "propellers positioned diagonally opposite each other typically rotate in the same direction."
        )
        self._direction = ParamGetterWidget_ComboBox(
            "Turning Direction", direction_description, [TurningDirection.CW.name, TurningDirection.CCW.name]
        )
        self._param_rows.addWidget(self._direction)

        kv_description = "Motor's rotational speed under no load, relative to the supplied voltage."
        self._kv = ParamGetterWidget_SpinBox(
            "Kv", kv_description, minimum=1, maximum=10 ** 5, default=920, suffix=" rpm/V"
        )
        self._param_rows.addWidget(self._kv)

        resistance_description = "Internal resistance value of the motor."
        self._resistance = ParamGetterWidget_SpinBox(
            "Internal Registance", resistance_description, minimum=1, default=250, suffix=" mΩ"
        )
        self._param_rows.addWidget(self._resistance)

        num_poles_description = ""  # TODO
        self._num_poles = ParamGetterWidget_SpinBox("Number of Poles", num_poles_description, minimum=2, default=14)
        self._param_rows.addWidget(self._num_poles)

        time_const_up_description = (
            "Time constant of the motor's response when increasing its rotational speed, "
            "relative to the command value."
        )
        self._time_const_up = ParamGetterWidget_SpinBox(
            "Time Constant Up", time_const_up_description, minimum=1, default=15, suffix=" ms"
        )
        self._param_rows.addWidget(self._time_const_up)

        time_const_down_description = (
            "Time constant of the motor's response when decreasing its rotational speed, "
            "relative to the command value."
        )
        self._time_const_down = ParamGetterWidget_SpinBox(
            "Time Constant Down", time_const_down_description, minimum=1, default=30, suffix=" ms"
        )
        self._param_rows.addWidget(self._time_const_down)

        # Max Rotation Speed
        self._max_rot_speed = MaxRotationSpeedWidget(main, link_name)
        rows.addWidget(self._max_rot_speed)

        rows.addStretch()

    @override
    def is_valid(self) -> bool:
        if self.num_poles() % 2 == 1:
            q_error_named(self, PROPULSION_SYSTEM, "The number of poles of a brushless motor must be even.")
            return False

        if not self._max_rot_speed.is_valid():
            return False

        return True

    @override
    def copy_from(self, src: MotorWidget) -> None:
        for i in range(self._param_rows.count()):
            param_des: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param_src: ParamGetterWidget = src._param_rows.itemAt(i).widget()
            param_des.set(param_src.get())

        self._max_rot_speed.copy_from(src._max_rot_speed)

    @override
    def dump_settings(self) -> dict:
        res = dict()

        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()

        res[self.MAX_ROT_SPEED_KEY] = self._max_rot_speed.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])

        self._max_rot_speed.load_settings(data[self.MAX_ROT_SPEED_KEY])

    def direction(self) -> str:
        """CW or CCW"""
        return self._direction.get()

    def kv(self) -> float:
        """[rad/s/V]"""
        return rpm2rps(self._kv.get())

    def internal_resistance(self) -> float:
        """[Ω]"""
        return self._resistance.get() * 1e-3

    def num_poles(self) -> int:
        return self._num_poles.get()

    def time_const_up(self) -> float:
        """[s]"""
        return self._time_const_up.get() * 1e-3

    def time_const_down(self) -> float:
        """[s]"""
        return self._time_const_down.get() * 1e-3

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        return self._max_rot_speed.max_rot_speed()
