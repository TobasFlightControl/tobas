from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

import math
from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from ...parameter_getters import ParamGetterWidget_SpinBox
from .base import BaseSelectedLinkSettingWidget


class PropellerWidget(BaseSelectedLinkSettingWidget):
    NAME = "Propeller"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._param_rows = QVBoxLayout()
        rows.addLayout(self._param_rows)

        num_blade_description = "Number of blades per propeller."
        self._num_blade = ParamGetterWidget_SpinBox("Number of blades", num_blade_description, minimum=1, default=2)
        self._param_rows.addWidget(self._num_blade)

        diameter_description = "Diameter of the propeller's rotational plane."
        self._diameter = ParamGetterWidget_SpinBox(
            "Propeller Diameter", diameter_description, minimum=1, default=10, suffix=" inch"
        )
        self._param_rows.addWidget(self._diameter)

        blade_chord_description = "Chord length at 75% of the distance from the blade's center."
        self._blade_chord = ParamGetterWidget_SpinBox(
            "75% Blade chord", blade_chord_description, minimum=1, default=15, suffix=" mm"
        )
        self._param_rows.addWidget(self._blade_chord)

        pitch_angle_description = "Twist angle at 75% of the distance from the blade's center."
        self._pitch_angle = ParamGetterWidget_SpinBox(
            "75% Blade pitch angle", pitch_angle_description, minimum=1, maximum=90, default=15, suffix=" deg"
        )
        self._param_rows.addWidget(self._pitch_angle)

        rows.addStretch()

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def copy_from(self, src: PropellerWidget) -> None:
        for i in range(self._param_rows.count()):
            param_des: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param_src: ParamGetterWidget = src._param_rows.itemAt(i).widget()
            param_des.set(param_src.get())

    @override
    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()
        return res

    @override
    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])

    def num_blade(self) -> int:
        return self._num_blade.get()

    def propeller_diameter(self) -> float:
        """[m]"""
        return self._diameter.get() * 0.0254

    def propeller_radius(self) -> float:
        """[m]"""
        return self.propeller_diameter() / 2

    def blade_chord(self) -> float:
        """[m]"""
        return self._blade_chord.get() / 1000

    def pitch_angle(self) -> float:
        """[rad]"""
        return math.radians(self._pitch_angle.get())
