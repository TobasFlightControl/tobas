from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from ...parameter_getters import (
    ParamGetterWidget_DoubleSpinBox,
    ParamGetterWidget_DoubleRange,
    ParamGetterWidget_Vector3d,
)

from .base import BaseFixedWingSettingWidget


class VehicleParametersWidget(BaseFixedWingSettingWidget):
    NAME = "Vehicle Parameters"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        wing_surface_description = ""
        self.wing_surface = ParamGetterWidget_DoubleSpinBox(
            "Wing Surface",
            wing_surface_description,
            decimals=3,
            minimum=0.001,
            default=0.47,
            suffix=" m^2",
        )
        self._param_rows.addWidget(self.wing_surface)

        wing_span_description = ""
        self.wing_span = ParamGetterWidget_DoubleSpinBox(
            "Wing Span",
            wing_span_description,
            decimals=3,
            minimum=0.001,
            default=2.59,
            suffix=" m",
        )
        self._param_rows.addWidget(self.wing_span)

        mac_description = ""
        self.mac = ParamGetterWidget_DoubleSpinBox(
            "Mean Aerodynamic Chord",
            mac_description,
            decimals=3,
            minimum=0.001,
            default=0.18,
            suffix=" m",
        )
        self._param_rows.addWidget(self.mac)

        aerodynamic_center_description = ""
        self.aerodynamic_center = ParamGetterWidget_Vector3d(
            "Aerodynamic Center",
            aerodynamic_center_description,
            decimals=3,
            default=(0.1, 0.0, 0.0),
            suffix=" m",
        )
        self._param_rows.addWidget(self.aerodynamic_center)

        alpha_limit_description = ""
        self.alpha_limit = ParamGetterWidget_DoubleRange(
            "Limitation of Angle of Attack",
            alpha_limit_description,
            decimals=3,
            default=(-0.27, 0.27),
            suffix=" rad",
        )
        self._param_rows.addWidget(self.alpha_limit)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        return True

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
