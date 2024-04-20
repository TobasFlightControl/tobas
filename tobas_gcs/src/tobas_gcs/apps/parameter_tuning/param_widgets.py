import math
from abc import abstractmethod
from overrides import override
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QLineEdit, QHBoxLayout
from PyQt5.QtGui import QIntValidator, QDoubleValidator

from tobas_rqt_tools.widgets import Slider, FloatSlider


class BaseParamWidget(QWidget):
    VALUE_WIDTH = 100

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def get(self):
        raise NotImplementedError()

    @abstractmethod
    def set(self, value):
        raise NotImplementedError()


class IntParamWidget(BaseParamWidget):
    def __init__(self, minimum: int, maximum: int) -> None:
        super().__init__()

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel(str(minimum)))

        self._slider = Slider(Qt.Horizontal)
        self._slider.setRange(minimum, maximum)
        self._slider.valueChanged.connect(self._on_slider_value_changed)
        cols.addWidget(self._slider)

        cols.addWidget(QLabel(str(maximum)))

        self._lineedit = QLineEdit()
        self._lineedit.setFixedWidth(self.VALUE_WIDTH)
        self._lineedit.setValidator(QIntValidator(minimum, maximum))
        self._lineedit.textChanged.connect(self._on_lineedit_text_changed)
        cols.addWidget(self._lineedit)

    @override
    def get(self):
        return self._slider.value()

    @override
    def set(self, value):
        self._set_slider_value(value)
        self._set_lineedit_text(value)

    @pyqtSlot(int)
    def _on_slider_value_changed(self, value: int) -> None:
        self._set_lineedit_text(value)

    @pyqtSlot(str)
    def _on_lineedit_text_changed(self, text: str) -> None:
        self._set_slider_value(int(text))

    def _set_slider_value(self, value: int) -> None:
        self._slider.blockSignals(True)
        self._slider.setValue(value)
        self._slider.blockSignals(False)

    def _set_lineedit_text(self, value: int) -> None:
        self._lineedit.blockSignals(True)
        self._lineedit.setText(f"{value}")
        self._lineedit.blockSignals(False)


class FloatParamWidget(BaseParamWidget):
    def __init__(self, minimum: float, maximum: float) -> None:
        super().__init__()

        range_digit = math.floor(math.log10(maximum - minimum))
        self._decimals = max(2 - range_digit, 0)

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel(format(minimum, f".{self._decimals}f")))

        self._slider = FloatSlider(Qt.Horizontal)
        self._slider.setRange(minimum, maximum)
        self._slider.valueChanged.connect(self._on_slider_value_changed)
        cols.addWidget(self._slider)

        cols.addWidget(QLabel(format(maximum, f".{self._decimals}f")))

        self._lineedit = QLineEdit()
        self._lineedit.setFixedWidth(self.VALUE_WIDTH)
        self._lineedit.setValidator(QDoubleValidator(minimum, maximum, self._decimals))
        self._lineedit.textChanged.connect(self._on_lineedit_text_changed)
        cols.addWidget(self._lineedit)

    @override
    def get(self):
        return self._slider.value()

    @override
    def set(self, value):
        self._set_slider_value(value)
        self._set_lineedit_text(value)

    @pyqtSlot(float)
    def _on_slider_value_changed(self, value: float) -> None:
        self._set_lineedit_text(value)

    @pyqtSlot(str)
    def _on_lineedit_text_changed(self, text: str) -> None:
        self._set_slider_value(float(text))

    def _set_slider_value(self, value: float) -> None:
        self._slider.blockSignals(True)
        self._slider.setValue(value)
        self._slider.blockSignals(False)

    def _set_lineedit_text(self, value: float) -> None:
        self._lineedit.blockSignals(True)
        self._lineedit.setText(format(value, f".{self._decimals}f"))
        self._lineedit.blockSignals(False)
