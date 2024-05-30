import math
from typing import Tuple, Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QHBoxLayout

from .base import ParamGetterWidget
from .utils import FloatGetter


class ParamGetterWidget_Pose(ParamGetterWidget):
    value_changed = pyqtSignal(float, float, float, float, float, float)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
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
        cols_xyz = QHBoxLayout()
        self._rows.addLayout(cols_xyz)

        self._x = FloatGetter("x", decimals, xyz_min[0], xyz_max[0], 0.1, xyz_default[0], xyz_suffix)
        cols_xyz.addWidget(self._x)

        self._y = FloatGetter("y", decimals, xyz_min[1], xyz_max[1], 0.1, xyz_default[1], xyz_suffix)
        cols_xyz.addWidget(self._y)

        self._z = FloatGetter("z", decimals, xyz_min[2], xyz_max[2], 0.1, xyz_default[2], xyz_suffix)
        cols_xyz.addWidget(self._z)

        # RPY
        cols_rpy = QHBoxLayout()
        self._rows.addLayout(cols_rpy)

        self._roll = FloatGetter("roll", decimals, rpy_min[0], rpy_max[0], 0.1, rpy_default[0], rpy_suffix)
        cols_rpy.addWidget(self._roll)

        self._pitch = FloatGetter("pitch", decimals, rpy_min[1], rpy_max[1], 0.1, rpy_default[1], rpy_suffix)
        cols_rpy.addWidget(self._pitch)

        self._yaw = FloatGetter("yaw", decimals, rpy_min[2], rpy_max[2], 0.1, rpy_default[2], rpy_suffix)
        cols_rpy.addWidget(self._yaw)

        self._x.value_changed.connect(self._on_value_changed)
        self._y.value_changed.connect(self._on_value_changed)
        self._z.value_changed.connect(self._on_value_changed)
        self._roll.value_changed.connect(self._on_value_changed)
        self._pitch.value_changed.connect(self._on_value_changed)
        self._yaw.value_changed.connect(self._on_value_changed)

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

    def set(self, src: Tuple[float, float, float, float, float, float]) -> None:
        self._x.set(src[0])
        self._y.set(src[1])
        self._z.set(src[2])
        self._roll.set(src[3])
        self._pitch.set(src[4])
        self._yaw.set(src[5])

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self.x(), self.y(), self.z(), self.roll(), self.pitch(), self.yaw())
