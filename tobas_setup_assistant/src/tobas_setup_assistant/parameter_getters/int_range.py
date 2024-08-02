from typing import override
from typing import Tuple, Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QHBoxLayout

from .base import ParamGetterWidget
from .utils import IntGetter


class ParamGetterWidget_IntRange(ParamGetterWidget[Tuple[int, int]]):
    value_changed = pyqtSignal(int, int)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
        minimum: int = -(10**9),
        maximum: int = +(10**9),
        single_step: int = 1,
        default: Tuple[int, int] = (0, 0),
        suffix: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._min = IntGetter("min", minimum, maximum, single_step, default[0], suffix)
        cols.addWidget(self._min)

        self._max = IntGetter("max", minimum, maximum, single_step, default[1], suffix)
        cols.addWidget(self._max)

        self._min.value_changed.connect(self._on_value_changed)
        self._max.value_changed.connect(self._on_value_changed)

    def min(self) -> int:
        return self._min.get()

    def max(self) -> int:
        return self._max.get()

    @override
    def get(self) -> Tuple[int, int]:
        return self.min(), self.max()

    @override
    def set(self, src: Tuple[int, int]) -> None:
        self._min.set(src[0])
        self._max.set(src[1])

    def is_valid(self) -> bool:
        return self.min() <= self.max()

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(self.min(), self.max())
