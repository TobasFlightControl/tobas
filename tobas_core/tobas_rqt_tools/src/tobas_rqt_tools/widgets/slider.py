from overrides import override
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QWidget, QSlider
from PyQt5.QtGui import QWheelEvent

from tobas_std_tools_py.math import remap


class Slider(QSlider):
    """
    ===== QSliderとの違い =====
    - マウスホイールイベントを無効化
    """

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class FloatSlider(Slider):
    """小数を扱うスライダー．"""

    RANGE = 10000
    DEFAULT_MINIMUM = 0.0
    DEFAULT_MAXIMUM = 1.0

    valueChanged = pyqtSignal(float)  # シグナルのオーバーライドも可能

    def __init__(self, orientation: Qt.Orientation) -> None:
        super().__init__(orientation)

        self._min = self.DEFAULT_MINIMUM
        self._max = self.DEFAULT_MAXIMUM

        super().setRange(0, self.RANGE)
        super().setValue(self.RANGE // 2)

        super().valueChanged.connect(self._on_slider_value_changed)

    @override
    def minimum(self) -> float:
        return self._min

    @override
    def setMinimum(self, minimum: float) -> None:
        self._min = minimum

    @override
    def maximum(self) -> float:
        return self._max

    @override
    def setMaximum(self, maximum: float) -> None:
        self._max = maximum

    @override
    def value(self) -> float:
        slider_value = super().value()
        return self._value_from_slider(slider_value)

    @override
    def setValue(self, value: float) -> None:
        slider_value = int(remap(value, self._min, self._max, 0.0, self.RANGE))
        super().setValue(slider_value)

    @pyqtSlot(int)
    def _on_slider_value_changed(self, slider_value: int) -> None:
        value = self._value_from_slider(slider_value)
        self.valueChanged.emit(value)

    def _value_from_slider(self, slider_value: int) -> float:
        return remap(float(slider_value), 0.0, self.RANGE, self._min, self._max)
