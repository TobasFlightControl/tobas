from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from ...parameter_getters import (
    ParamGetterWidget_DoubleSpinBox,
    ParamGetterWidget_DoubleRange,
    ParamGetterWidget_Vector3d,
)


class VehicleParametersWidget(QWidget):
    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        wing_surface_description = ""
        self.wing_surface = ParamGetterWidget_DoubleSpinBox(
            "Wing Surface", wing_surface_description, decimals=3, minimum=0.001, default=0.47, suffix=" m^2"
        )
        self._rows.addWidget(self.wing_surface)

        wing_span_description = ""
        self.wing_span = ParamGetterWidget_DoubleSpinBox(
            "Wing Span", wing_span_description, decimals=3, minimum=0.001, default=2.59, suffix=" m"
        )
        self._rows.addWidget(self.wing_span)

        mac_description = ""
        self.mac = ParamGetterWidget_DoubleSpinBox(
            "Mean Aerodynamic Chord", mac_description, decimals=3, minimum=0.001, default=0.18, suffix=" m"
        )
        self._rows.addWidget(self.mac)

        aerodynamic_center_description = ""
        self.aerodynamic_center = ParamGetterWidget_Vector3d(
            "Aerodynamic Center", aerodynamic_center_description, decimals=3, default=(0.1, 0.0, 0.0), suffix=" m"
        )
        self._rows.addWidget(self.aerodynamic_center)

        alpha_limit_description = ""
        self.alpha_limit = ParamGetterWidget_DoubleRange(
            "Limitation of Angle of Attack", alpha_limit_description, decimals=3, default=(-0.27, 0.27), suffix=" rad"
        )
        self._rows.addWidget(self.alpha_limit)

    def is_valid(self) -> bool:
        return True

    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._rows.count()):
            param: ParamGetterWidget = self._rows.itemAt(i).widget()
            res[param.name()] = param.get()
        return res

    def load_settings(self, data: dict) -> None:
        for i in range(self._rows.count()):
            param: ParamGetterWidget = self._rows.itemAt(i).widget()
            param.set(data[param.name()])
