from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import add_spacer

from .base_setting import BaseSettingWidget
from ..common import *
from ..parameter_getters import *


class RCTransmitterWidget(BaseSettingWidget):
    NAME = "RC Transmitter"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define RC Transmitter"
        abst_text = "Configure the RC transmitter settings. Please set appropriate values for each item."
        super().__init__(main, title_text, abst_text)

        num_modes_description = ""
        self.num_modes = ParamGetterWidget_SpinBox(
            "The number of flight modes (CH5)",
            num_modes_description,
            minimum=1,
            maximum=6,
            default=DEFAULT_NUM_FLIGHT_MODES,
        )
        self._rows.addWidget(self.num_modes)

        dead_zone_rate_description = ""
        self.dead_zone_rate = ParamGetterWidget_SpinBox(
            "Dead zone rate",
            dead_zone_rate_description,
            minimum=0,
            maximum=30,
            default=10,
            suffix=" %",
        )
        self._rows.addWidget(self.dead_zone_rate)

        add_spacer(self._rows)

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self.num_modes.value_changed.connect(self._on_num_modes_changed)

    @overrides
    def is_valid(self) -> bool:
        return True

    @pyqtSlot(int)
    def _on_num_modes_changed(self, num_modes: int) -> None:
        self._main.signals.num_modes_updated.emit(num_modes)
