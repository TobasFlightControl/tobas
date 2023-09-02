import math
from typing import Tuple
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget
from .utils import DoubleGetter


class ParamGetterWidget_Pose(ParamGetterWidget):
    value_changed = pyqtSignal(float, float, float, float, float, float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        decimals: int = 3,
        xyz_min: Tuple[float, float, float] = (-1e9,) * 3,
        xyz_max: Tuple[float, float, float] = (+1e9,) * 3,
        xyz_default: Tuple[float, float, float] = (0.0,) * 3,
        xyz_suffix: str = " m",
        rpy_min: Tuple[float, float, float] = (-math.pi,) * 3,
        rpy_max: Tuple[float, float, float] = (+math.pi,) * 3,
        rpy_default: Tuple[float, float, float] = (0.0,) * 3,
        rpy_suffix: str = " rad",
    ) -> None:
        super().__init__(param_name, description_text)

        # XYZ
        self._cols_xyz = QHBoxLayout()
        self._rows.addLayout(self._cols_xyz)

        self._x = DoubleGetter(
            "x", decimals, xyz_min[0], xyz_max[0], 0.1, xyz_default[0], xyz_suffix
        )
        self._cols_xyz.addWidget(self._x)

        self._y = DoubleGetter(
            "y", decimals, xyz_min[1], xyz_max[1], 0.1, xyz_default[1], xyz_suffix
        )
        self._cols_xyz.addWidget(self._y)

        self._z = DoubleGetter(
            "z", decimals, xyz_min[2], xyz_max[2], 0.1, xyz_default[2], xyz_suffix
        )
        self._cols_xyz.addWidget(self._z)

        # RPY
        self._cols_rpy = QHBoxLayout()
        self._rows.addLayout(self._cols_rpy)

        self._roll = DoubleGetter(
            "roll", decimals, rpy_min[0], rpy_max[0], 0.1, rpy_default[0], rpy_suffix
        )
        self._cols_rpy.addWidget(self._roll)

        self._pitch = DoubleGetter(
            "pitch", decimals, rpy_min[1], rpy_max[1], 0.1, rpy_default[1], rpy_suffix
        )
        self._cols_rpy.addWidget(self._pitch)

        self._yaw = DoubleGetter(
            "yaw", decimals, rpy_min[2], rpy_max[2], 0.1, rpy_default[2], rpy_suffix
        )
        self._cols_rpy.addWidget(self._yaw)

        self._x.data.valueChanged.connect(self._on_value_changed)
        self._y.data.valueChanged.connect(self._on_value_changed)
        self._z.data.valueChanged.connect(self._on_value_changed)
        self._roll.data.valueChanged.connect(self._on_value_changed)
        self._pitch.data.valueChanged.connect(self._on_value_changed)
        self._yaw.data.valueChanged.connect(self._on_value_changed)

    def x(self) -> float:
        return self._x.get()

    def y(self) -> float:
        return self._y.get()

    def z(self) -> float:
        return self._z.get()

    def roll(self) -> float:
        return self._roll.get()

    def pitch(self) -> float:
        return self._pitch.get()

    def yaw(self) -> float:
        return self._yaw.get()

    def get(self) -> Tuple[float, float, float, float, float, float]:
        return self.x(), self.y(), self.z(), self.roll(), self.pitch(), self.yaw()

    def set(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> None:
        self._x.data.setValue(x)
        self._y.data.setValue(y)
        self._z.data.setValue(z)
        self._roll.data.setValue(roll)
        self._pitch.data.setValue(pitch)
        self._yaw.data.setValue(yaw)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(
            self.x(), self.y(), self.z(), self.roll(), self.pitch(), self.yaw()
        )
