from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..common import *
from ..parameter_getters import *


class BarometerWidget(BaseSettingWidget):
    NAME = "Barometer"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Air Pressure Sensor"
        abst_text = ""
        super().__init__(main, title_text, abst_text)

        self.offset = ParamGetterWidget_Vector3d(
            "Offset",
            SENSOR_OFFSET_DESCRIPTION,
            suffix=" m",
        )
        self._rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=50,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        pressure_var_description = ""
        self.pressure_var = ParamGetterWidget_DoubleSpinBox(
            "the air pressure variance",
            pressure_var_description,
            decimals=2,
            minimum=0.0,
            default=10.0,
            suffix=" Pa^2",
        )
        self._rows.addWidget(self.pressure_var)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()

    @override
    def is_valid(self) -> bool:
        return True

    def equipped(self) -> bool:
        return True
