from overrides import override
from typing import Tuple, Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QHBoxLayout

from .base import ParamGetterWidget
from .utils import FloatGetter


class ParamGetterWidget_DoubleRange(ParamGetterWidget):
    value_changed = pyqtSignal(float, float)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
        decimals: int = 3,
        minimum: float = -1e9,
        maximum: float = +1e9,
        single_step: float = 1.0,
        default: Tuple[float, float] = (0.0, 0.0),
        suffix: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._min = FloatGetter("min", decimals, minimum, maximum, single_step, default[0], suffix)
        cols.addWidget(self._min)

        self._max = FloatGetter("max", decimals, minimum, maximum, single_step, default[1], suffix)
        cols.addWidget(self._max)

        self._min.value_changed.connect(self._on_value_changed)
        self._max.value_changed.connect(self._on_value_changed)

    @override
    def get(self) -> Tuple[float, float]:
        return self.min(), self.max()

    @override
    def set(self, src: Tuple[float, float]) -> None:
        self._min.set(src[0])
        self._max.set(src[1])

    def min(self) -> float:
        return self._min.get()

    def max(self) -> float:
        return self._max.get()

    def is_valid(self) -> bool:
        return self.min() <= self.max()

    @pyqtSlot(float)
    def _on_value_changed(self, _: float) -> None:
        self.value_changed.emit(self.min(), self.max())
