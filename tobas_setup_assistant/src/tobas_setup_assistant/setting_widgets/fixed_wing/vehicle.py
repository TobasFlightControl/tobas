from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ...parameter_getters import *
from ...common import *


class VehicleParametersWidget(QWidget):
    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Vehicle Parameters")
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        rows.addWidget(label)

        wing_surface_description = ""
        self.wing_surface = ParamGetterWidget_DoubleSpinBox(
            "Wing Surface",
            wing_surface_description,
            decimals=3,
            minimum=0.001,
            default=0.47,
            suffix=" m^2",
        )
        rows.addWidget(self.wing_surface)

        wing_span_description = ""
        self.wing_span = ParamGetterWidget_DoubleSpinBox(
            "Wing Span",
            wing_span_description,
            decimals=3,
            minimum=0.001,
            default=2.59,
            suffix=" m",
        )
        rows.addWidget(self.wing_span)

        mac_description = ""
        self.mac = ParamGetterWidget_DoubleSpinBox(
            "Mean Aerodynamic Chord",
            mac_description,
            decimals=3,
            minimum=0.001,
            default=0.18,
            suffix=" m",
        )
        rows.addWidget(self.mac)

        aerodynamic_center_description = ""
        self.aerodynamic_center = ParamGetterWidget_Vector3d(
            "Aerodynamic Center",
            aerodynamic_center_description,
            decimals=3,
            default=(0.1, 0.0, 0.0),
            suffix=" m",
        )
        rows.addWidget(self.aerodynamic_center)

        alpha_limit_description = ""
        self.alpha_limit = ParamGetterWidget_DoubleRange(
            "Limitation of Angle of Attack",
            alpha_limit_description,
            decimals=3,
            default=(-0.27, 0.27),
            suffix=" rad",
        )
        rows.addWidget(self.alpha_limit)

    def define_connections(self) -> None:
        pass

    def is_valid(self) -> bool:
        return True
