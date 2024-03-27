from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget

from ...parameter_getters import *
from ...common import *


class EscWidget(Widget):
    SIGNAL_MODE_MAP = {
        "BLHeli Open Loop": "blheli_open_loop",
        "BHLeli Closed Loop (Low Range)": "blheli_closed_loop_low_range",
        "BHLeli Closed Loop (Middle Range)": "blheli_closed_loop_mid_range",
        "BHLeli Closed Loop (High Range)": "blheli_closed_loop_high_range",
    }

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        rows = QVBoxLayout()
        self.setLayout(rows)

        title = QLabel("ESC Settings")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        rows.addWidget(title)

        max_current_description = (
            "Maximum current that the ESC (Electronic Speed Controller) can safely handle. "
            "Exceeding this maximum current may lead to overheating or damage to the ESC, "
            "and in the worst case, it could cause failure or fire."
        )
        self._max_current = ParamGetterWidget_SpinBox(
            "Maximum Current", max_current_description, minimum=1, default=20, suffix=" A"
        )
        rows.addWidget(self._max_current)

        signal_mode_description = ""  # TODO
        self._signal_mode = ParamGetterWidget_ComboBox(
            "Signal Mode", signal_mode_description, self.SIGNAL_MODE_MAP.keys()
        )
        rows.addWidget(self._signal_mode)

    def is_valid(self) -> bool:
        return True

    def copy_from(self, src: EscWidget) -> None:
        self._max_current.set(src._max_current.get())
        self._signal_mode.set(src._signal_mode.get())

    def max_current(self) -> float:
        return self._max_current.get()

    def signal_mode(self) -> str:
        return self.SIGNAL_MODE_MAP[self._signal_mode.get()]
