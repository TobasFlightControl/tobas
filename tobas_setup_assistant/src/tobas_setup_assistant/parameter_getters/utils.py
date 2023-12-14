from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import SpinBox, DoubleSpinBox

from ..common import *


class IntGetter(QWidget):
    value_changed = pyqtSignal(int)

    def __init__(
        self,
        name: str,
        minimum: int,
        maximum: int,
        single_step: int,
        default: int,
        suffix: str,
    ) -> None:
        assert minimum <= maximum
        assert single_step > 0

        super().__init__()

        cols = QHBoxLayout()
        self.setLayout(cols)

        label = QLabel(name + ":")
        label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        cols.addWidget(label)

        self._data = SpinBox()
        self._data.setMinimum(minimum)
        self._data.setMaximum(maximum)
        self._data.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self._data.setValue(default)
        self._data.setSuffix(suffix)
        self._data.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        cols.addWidget(self._data)

        self._data.valueChanged.connect(self._on_value_changed)

    def get(self) -> int:
        return self._data.value()

    def set(self, value: int) -> None:
        self._data.setValue(value)

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(value)


class FloatGetter(QWidget):
    value_changed = pyqtSignal(float)

    def __init__(
        self,
        name: str,
        decimals: int,
        minimum: float,
        maximum: float,
        single_step: float,
        default: float,
        suffix: str,
    ) -> None:
        assert decimals >= 0
        assert minimum < maximum
        assert single_step > 0.0

        super().__init__()

        cols = QHBoxLayout()
        self.setLayout(cols)

        label = QLabel(name + ":")
        label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        cols.addWidget(label)

        self._data = DoubleSpinBox()
        self._data.setDecimals(decimals)
        self._data.setMinimum(minimum)
        self._data.setMaximum(maximum)
        self._data.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self._data.setValue(default)
        self._data.setSuffix(suffix)
        self._data.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        cols.addWidget(self._data)

        self._data.valueChanged.connect(self._on_value_changed)

    def get(self) -> float:
        return self._data.value()

    def set(self, value: float) -> None:
        self._data.setValue(value)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(value)
