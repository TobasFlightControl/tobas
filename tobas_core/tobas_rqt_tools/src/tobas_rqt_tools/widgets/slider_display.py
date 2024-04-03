import random
from typing import Callable
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtWidgets import QWidget, QLabel, QLineEdit, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QFont

from .slider import Slider, FloatSlider


class IntSliderDisplay(QWidget):
    """整数スライダーとその値の表示機能を持つウィジェット．"""

    PSIZE = 9

    value_changed = pyqtSignal(int)

    def __init__(self) -> None:
        super().__init__()
        self._suffix = ""

        font = QFont("Default", self.PSIZE, QFont.Bold)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._text = QLabel()
        self._text.setFont(font)
        cols.addWidget(self._text)

        self._value = QLineEdit()
        self._value.setAlignment(Qt.AlignRight)
        self._value.setFont(font)
        self._value.setReadOnly(True)
        self._value.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._value)

        self._slider = Slider(Qt.Horizontal)
        rows.addWidget(self._slider)

        self.update()
        self._slider.valueChanged.connect(self._on_value_changed)

    def update(self) -> None:
        self._value.setText(f"{self.get_value()}{self._suffix}")
        self.value_changed.emit(self.get_value())

    def set_text(self, text: str) -> None:
        self._text.setText(text)

    def set_minimum(self, minimum: int) -> None:
        self._slider.setMinimum(minimum)

    def set_maximum(self, maximum: int) -> None:
        self._slider.setMaximum(maximum)

    def get_value(self) -> int:
        return self._slider.value()

    def set_value(self, value: int) -> None:
        self._slider.setValue(value)

    def set_suffix(self, suffix: str) -> None:
        self._suffix = suffix
        self.update()

    def set_callback(self, callback: Callable[[int], None]):
        self.value_changed.connect(callback)

    def set_random_value(self) -> None:
        value = random.randint(self._slider.minimum(), self._slider.maximum())
        self.set_value(value)

    def set_center_value(self) -> None:
        value = (self._slider.minimum() + self._slider.maximum()) // 2
        self.set_value(value)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self.update()


class FloatSliderDisplay(QWidget):
    """小数スライダーとその値の表示機能を持つウィジェット．"""

    PSIZE = 9

    value_changed = pyqtSignal(float)

    def __init__(self) -> None:
        super().__init__()
        self._suffix = ""

        font = QFont("Default", self.PSIZE, QFont.Bold)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._text = QLabel()
        self._text.setFont(font)
        cols.addWidget(self._text)

        self._value = QLineEdit()
        self._value.setAlignment(Qt.AlignRight)
        self._value.setFont(font)
        self._value.setReadOnly(True)
        self._value.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._value)

        self._slider = FloatSlider(Qt.Horizontal)
        rows.addWidget(self._slider)

        self.update()
        self._slider.valueChanged.connect(self._on_value_changed)

    def update(self) -> None:
        self._value.setText(f"{self.get_value()}{self._suffix}")
        self.value_changed.emit(self.get_value())

    def set_text(self, text: str) -> None:
        self._text.setText(text)

    def set_minimum(self, minimum: float) -> None:
        self._slider.setMinimum(minimum)

    def set_maximum(self, maximum: float) -> None:
        self._slider.setMaximum(maximum)

    def get_value(self) -> float:
        return self._slider.value()

    def set_value(self, value: float) -> None:
        self._slider.setValue(value)

    def set_suffix(self, suffix: str) -> None:
        self._suffix = suffix

    def set_callback(self, callback: Callable[[float], None]) -> None:
        self.value_changed.connect(callback)

    def set_random_value(self) -> None:
        value = random.uniform(self._slider.minimum(), self._slider.maximum())
        self.set_value(value)

    def set_center_value(self) -> None:
        value = (self._slider.minimum() + self._slider.maximum()) / 2
        self.set_value(value)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self.update()
