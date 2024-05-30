from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

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

        rows = QVBoxLayout()
        self.setLayout(rows)

        wing_surface_description = ""
        self.wing_surface = ParamGetterWidget_DoubleSpinBox(
            "Wing Surface", wing_surface_description, decimals=3, minimum=0.001, default=0.47, suffix=" m^2"
        )
        rows.addWidget(self.wing_surface)

        wing_span_description = ""
        self.wing_span = ParamGetterWidget_DoubleSpinBox(
            "Wing Span", wing_span_description, decimals=3, minimum=0.001, default=2.59, suffix=" m"
        )
        rows.addWidget(self.wing_span)

        mac_description = ""
        self.mac = ParamGetterWidget_DoubleSpinBox(
            "Mean Aerodynamic Chord", mac_description, decimals=3, minimum=0.001, default=0.18, suffix=" m"
        )
        rows.addWidget(self.mac)

        aerodynamic_center_description = ""
        self.aerodynamic_center = ParamGetterWidget_Vector3d(
            "Aerodynamic Center", aerodynamic_center_description, decimals=3, default=(0.1, 0.0, 0.0), suffix=" m"
        )
        rows.addWidget(self.aerodynamic_center)

        alpha_limit_description = ""
        self.alpha_limit = ParamGetterWidget_DoubleRange(
            "Limitation of Angle of Attack", alpha_limit_description, decimals=3, default=(-0.27, 0.27), suffix=" rad"
        )
        rows.addWidget(self.alpha_limit)

    def is_valid(self) -> bool:
        return True

    def dump_settings(self) -> dict:
        res = dict()

        res[self.wing_surface.name()] = self.wing_surface.get()
        res[self.wing_span.name()] = self.wing_span.get()
        res[self.mac.name()] = self.mac.get()
        res[self.aerodynamic_center.name()] = self.aerodynamic_center.get()
        res[self.alpha_limit.name()] = self.alpha_limit.get()

        return res

    def load_settings(self, data: dict) -> None:
        self.wing_surface.set(data[self.wing_surface.name()])
        self.wing_span.set(data[self.wing_span.name()])
        self.mac.set(data[self.mac.name()])
        self.aerodynamic_center.set(*data[self.aerodynamic_center.name()])
        self.alpha_limit.set(*data[self.alpha_limit.name()])
