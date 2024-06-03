from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from ...parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_ComboBox
from .base import BaseSelectedLinkSettingWidget


class EscWidget(BaseSelectedLinkSettingWidget):
    NAME = "ESC"

    SIGNAL_MODE_MAP = {
        "BLHeli Open Loop": "blheli_open_loop",
        "BHLeli Closed Loop (Low Range)": "blheli_closed_loop_low_range",
        "BHLeli Closed Loop (Middle Range)": "blheli_closed_loop_mid_range",
        "BHLeli Closed Loop (High Range)": "blheli_closed_loop_high_range",
    }

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._param_rows = QVBoxLayout()
        rows.addLayout(self._param_rows)

        max_current_description = (
            "Maximum current that the ESC (Electronic Speed Controller) can safely handle. "
            "Exceeding this maximum current may lead to overheating or damage to the ESC, "
            "and in the worst case, it could cause failure or fire."
        )
        self._max_current = ParamGetterWidget_SpinBox(
            "Maximum Current", max_current_description, minimum=1, default=20, suffix=" A"
        )
        self._param_rows.addWidget(self._max_current)

        signal_mode_description = ""  # TODO
        self._signal_mode = ParamGetterWidget_ComboBox(
            "Signal Mode", signal_mode_description, self.SIGNAL_MODE_MAP.keys()
        )
        self._param_rows.addWidget(self._signal_mode)

        rows.addStretch()

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def copy_from(self, src: EscWidget) -> None:
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

    def max_current(self) -> float:
        return self._max_current.get()

    def signal_mode(self) -> str:
        return self.SIGNAL_MODE_MAP[self._signal_mode.get()]
