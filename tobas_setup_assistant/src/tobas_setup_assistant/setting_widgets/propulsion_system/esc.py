from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ...parameter_getters import *
from ...common import *


class EscWidget(QWidget):
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
            "Maximum Current",
            max_current_description,
            minimum=1,
            default=20,
            suffix=" A",
        )
        rows.addWidget(self._max_current)

    def is_valid(self) -> bool:
        return True

    def copy_from(self, src: EscWidget) -> None:
        self._max_current.set(src._max_current.get())

    def max_current(self) -> float:
        return self._max_current.get()
