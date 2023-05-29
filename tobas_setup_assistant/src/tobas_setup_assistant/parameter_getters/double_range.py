from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from typing import Tuple

from .base import ParamGetterWidget
from .utils import DoubleGetter


class ParamGetterWidget_DoubleRange(ParamGetterWidget):

    value_changed = pyqtSignal(float, float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        decimals: int = 3,
        minimum: float = -1e+9,
        maximum: float = +1e+9,
        single_step: float = 1.,
        default: Tuple[float, float] = (0., 0.),
        suffix: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._min = DoubleGetter("min", decimals, minimum, maximum, single_step, default[0], suffix)
        self._cols.addWidget(self._min)

        self._max = DoubleGetter("max", decimals, minimum, maximum, single_step, default[1], suffix)
        self._cols.addWidget(self._max)

        self._min.data.valueChanged.connect(self._on_value_changed)
        self._max.data.valueChanged.connect(self._on_value_changed)

    def min(self) -> float:
        return self._min.get()

    def max(self) -> float:
        return self._max.get()

    def get(self) -> Tuple[float, float]:
        return self.min(), self.max()

    def set(self, min_: float, max_: float) -> None:
        self._min.data.setValue(min_)
        self._max.data.setValue(max_)

    def is_valid(self) -> bool:
        return self.min() <= self.max()

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self.min(), self.max())
