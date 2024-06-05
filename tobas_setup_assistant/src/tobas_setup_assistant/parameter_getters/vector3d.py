from overrides import override
from typing import Tuple, Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QHBoxLayout

from .base import ParamGetterWidget
from .utils import FloatGetter


class ParamGetterWidget_Vector3d(ParamGetterWidget[Tuple[float, float, float]]):
    value_changed = pyqtSignal(float, float, float)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
        decimals: int = 3,
        minimum: Tuple[float, float, float] = (-1e9,) * 3,
        maximum: Tuple[float, float, float] = (+1e9,) * 3,
        single_step: Tuple[float, float, float] = (1.0,) * 3,
        default: Tuple[float, float, float] = (0.0,) * 3,
        suffix: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._x = FloatGetter("x", decimals, minimum[0], maximum[0], single_step[0], default[0], suffix)
        cols.addWidget(self._x)

        self._y = FloatGetter("y", decimals, minimum[1], maximum[1], single_step[1], default[1], suffix)
        cols.addWidget(self._y)

        self._z = FloatGetter("z", decimals, minimum[2], maximum[2], single_step[2], default[2], suffix)
        cols.addWidget(self._z)

        self._x.value_changed.connect(self._on_value_changed)
        self._y.value_changed.connect(self._on_value_changed)
        self._z.value_changed.connect(self._on_value_changed)

    @override
    def get(self) -> Tuple[float, float, float]:
        return self.x(), self.y(), self.z()

    @override
    def set(self, src: Tuple[float, float, float]) -> None:
        self._x.set(src[0])
        self._y.set(src[1])
        self._z.set(src[2])

    def x(self) -> float:
        return self._x.get()

    def y(self) -> float:
        return self._y.get()

    def z(self) -> float:
        return self._z.get()

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self.x(), self.y(), self.z())
