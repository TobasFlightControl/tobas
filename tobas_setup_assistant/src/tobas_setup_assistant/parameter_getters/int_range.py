from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from typing import Tuple

from .base import ParamGetterWidget
from .utils import IntGetter


class ParamGetterWidget_IntRange(ParamGetterWidget):
    value_changed = pyqtSignal(int, int)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        minimum: int = -(10**9),
        maximum: int = +(10**9),
        single_step: int = 1,
        default: Tuple[int, int] = (0, 0),
        suffix: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._min = IntGetter("min", minimum, maximum, single_step, default[0], suffix)
        self._cols.addWidget(self._min)

        self._max = IntGetter("max", minimum, maximum, single_step, default[1], suffix)
        self._cols.addWidget(self._max)

        self._min.data.valueChanged.connect(self._on_value_changed)
        self._max.data.valueChanged.connect(self._on_value_changed)

    def min(self) -> int:
        return self._min.get()

    def max(self) -> int:
        return self._max.get()

    def get(self) -> Tuple[int, int]:
        return self.min(), self.max()

    def set(self, min_: int, max_: int) -> None:
        self._min.data.setValue(min_)
        self._max.data.setValue(max_)

    def is_valid(self) -> bool:
        return self.min() <= self.max()

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(self.min(), self.max())
