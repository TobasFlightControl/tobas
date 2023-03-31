from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class BatteryWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Battery'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        voltage_description = "TODO: instruction"
        self.voltage = ParamGetterWidget_DoubleSpinBox(
            "Voltage",
            voltage_description,
            decimals=1,
            minimum=0.,
            default=14.8,
            suffix=" V",
        )
        self._rows.addWidget(self.voltage)

        capacity_description = "TODO: instruction"
        self.capacity = ParamGetterWidget_SpinBox(
            "Current Capacity",
            capacity_description,
            minimum=1,
            default=5000,
            suffix=" mAh",
        )
        self._rows.addWidget(self.capacity)

        C_cont_description = "TODO: instruction"
        self.C_cont = ParamGetterWidget_SpinBox(
            "Continuous Discharge Current Rate",
            C_cont_description,
            minimum=1,
            default=50,
            suffix=" /h",
        )
        self._rows.addWidget(self.C_cont)

        C_pulse_description = "TODO: instruction"
        self.C_pulse = ParamGetterWidget_SpinBox(
            "Pulse Discharge Current Rate",
            C_pulse_description,
            minimum=1,
            default=100,
            suffix=" /h",
        )
        self._rows.addWidget(self.C_pulse)

        self._add_dummy_widget()
