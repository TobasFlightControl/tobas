import math
from abc import abstractmethod
from overrides import override
from typing import TypeVar, Generic
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QLineEdit, QHBoxLayout
from PyQt5.QtGui import QIntValidator, QDoubleValidator

from .slider import Slider, FloatSlider

T = TypeVar("T")


class SliderTextWidget(QWidget, Generic[T]):
    VALUE_WIDTH = 100

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def get(self) -> T:
        raise NotImplementedError()

    @abstractmethod
    def set(self, value: T) -> None:
        raise NotImplementedError()


class IntSliderTextWidget(SliderTextWidget[int]):
    value_changed = pyqtSignal(int)

    def __init__(self, minimum: int, maximum: int) -> None:
        super().__init__()

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel(str(minimum)))

        self._slider = Slider(Qt.Horizontal)
        self._slider.setRange(minimum, maximum)
        self._slider.sliderReleased.connect(self._on_slider_released)
        cols.addWidget(self._slider)

        cols.addWidget(QLabel(str(maximum)))

        self._lineedit = QLineEdit()
        self._lineedit.setFixedWidth(self.VALUE_WIDTH)
        self._lineedit.setValidator(QIntValidator(minimum, maximum))
        self._lineedit.returnPressed.connect(self._on_lineedit_return_pressed)
        cols.addWidget(self._lineedit)

    @override
    def get(self) -> int:
        return self._slider.value()

    @override
    def set(self, value: int) -> None:
        self._set_slider_value(value)
        self._set_lineedit_text(value)

    @pyqtSlot()
    def _on_slider_released(self) -> None:
        value = self._slider.value()
        self._set_lineedit_text(value)
        self.value_changed.emit(value)

    @pyqtSlot()
    def _on_lineedit_return_pressed(self) -> None:
        value = int(self._lineedit.text())
        self._set_slider_value(value)
        self.value_changed.emit(value)

    def _set_slider_value(self, value: int) -> None:
        self._slider.blockSignals(True)
        self._slider.setValue(value)
        self._slider.blockSignals(False)

    def _set_lineedit_text(self, value: int) -> None:
        self._lineedit.blockSignals(True)
        self._lineedit.setText(f"{value}")
        self._lineedit.blockSignals(False)


class FloatSliderTextWidget(SliderTextWidget[float]):
    value_changed = pyqtSignal(float)

    def __init__(self, minimum: float, maximum: float) -> None:
        super().__init__()

        range_digit = math.floor(math.log10(maximum - minimum))
        self._decimals = max(2 - range_digit, 0)

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel(format(minimum, f".{self._decimals}f")))

        self._slider = FloatSlider(Qt.Horizontal)
        self._slider.setRange(minimum, maximum)
        self._slider.sliderReleased.connect(self._on_slider_released)
        cols.addWidget(self._slider)

        cols.addWidget(QLabel(format(maximum, f".{self._decimals}f")))

        self._lineedit = QLineEdit()
        self._lineedit.setFixedWidth(self.VALUE_WIDTH)
        self._lineedit.setValidator(QDoubleValidator(minimum, maximum, self._decimals))
        self._lineedit.returnPressed.connect(self._on_lineedit_return_pressed)
        cols.addWidget(self._lineedit)

    @override
    def get(self) -> float:
        return self._slider.value()

    @override
    def set(self, value: float) -> None:
        self._set_slider_value(value)
        self._set_lineedit_text(value)

    @pyqtSlot()
    def _on_slider_released(self) -> None:
        value = self._slider.value()
        self._set_lineedit_text(value)
        self.value_changed.emit(value)

    @pyqtSlot()
    def _on_lineedit_return_pressed(self) -> None:
        value = float(self._lineedit.text())
        self._set_slider_value(value)
        self._set_lineedit_text(value)  # フォーマットを整える
        self.value_changed.emit(value)

    def _set_slider_value(self, value: float) -> None:
        self._slider.blockSignals(True)
        self._slider.setValue(value)
        self._slider.blockSignals(False)

    def _set_lineedit_text(self, value: float) -> None:
        self._lineedit.blockSignals(True)
        self._lineedit.setText(format(value, f".{self._decimals}f"))
        self._lineedit.blockSignals(False)
