from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ...parameter_getters import *
from ...constants import *


class BladeGeometry(QWidget):

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel("Blade Geometry")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        num_blade_description = "1つのプロペラに対するブレードの個数．"
        self._num_blade = ParamGetterWidget_SpinBox(
            "Number of blades",
            num_blade_description,
            minimum=1,
            default=2,
        )
        self._rows.addWidget(self._num_blade)

        diameter_description = "プロペラの回転面の直径．"
        self._diameter = ParamGetterWidget_SpinBox(
            "Propeller Diameter",
            diameter_description,
            minimum=1,
            default=200,
            suffix=" mm",
        )
        self._rows.addWidget(self._diameter)

        blade_chord_description = "ブレードの中心から75%の位置での弦長．"
        self._blade_chord = ParamGetterWidget_SpinBox(
            "75% Blade chord",
            blade_chord_description,
            minimum=1,
            default=15,
            suffix=" mm",
        )
        self._rows.addWidget(self._blade_chord)

        pitch_angle_description = "ブレードの中心から75%の位置でのねじれ角．"
        self._pitch_angle = ParamGetterWidget_SpinBox(
            "75% Blade pitch angle",
            pitch_angle_description,
            minimum=1,
            maximum=90,
            default=10,
            suffix=" deg",
        )
        self._rows.addWidget(self._pitch_angle)

    def is_valid(self) -> bool:
        return True

    def copy_from(self, src: BladeGeometry) -> None:
        self._num_blade.set(src._num_blade.get())
        self._diameter.set(src._diameter.get())
        self._blade_chord.set(src._blade_chord.get())
        self._pitch_angle.set(src._pitch_angle.get())

    def num_blade(self) -> int:
        return self._num_blade.get()

    def propeller_diameter(self) -> float:
        """ [m] """
        return self._diameter.get() / 1000.

    def propeller_radius(self) -> float:
        """ [m] """
        return self.propeller_diameter() / 2

    def blade_chord(self) -> float:
        """ [m] """
        return self._blade_chord.get() / 1000.

    def pitch_angle(self) -> float:
        """ [rad] """
        return math.radians(self._pitch_angle.get())
