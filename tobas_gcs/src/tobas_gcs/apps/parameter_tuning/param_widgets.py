from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout

from tobas_rqt_tools.widgets import FramedLabel, Slider, FloatSlider


class BaseParamWidget(QWidget):
    def __init__(self, name: str) -> None:
        super().__init__()

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._cols.addWidget(QLabel(name))


class IntParamWidget(BaseParamWidget):
    def __init__(self, name: str, minimum: int, maximum: int) -> None:
        super().__init__(name)

        self._slider = Slider(Qt.Horizontal)
        self._value = FramedLabel()

        self._cols.addWidget(QLabel(str(minimum)))
        self._cols.addWidget(self._slider)
        self._cols.addWidget(QLabel(str(maximum)))
        self._cols.addWidget(self._value)

        self._slider.valueChanged.connect(self._on_slider_value_changed)

    def get_value(self) -> int:
        return self._slider.value()

    def set_value(self, value: float) -> None:
        self._slider.setValue(value)

    @pyqtSlot()
    def _on_slider_value_changed(self) -> None:
        self._value.setText(f"{self.get_value()}")


class FloatParamWidget(BaseParamWidget):
    def __init__(self, name: str, minimum: float, maximum: float) -> None:
        super().__init__(name)

        self._slider = FloatSlider(Qt.Horizontal)
        self._value = FramedLabel()

        self._cols.addWidget(QLabel(str(minimum)))
        self._cols.addWidget(self._slider)
        self._cols.addWidget(QLabel(str(maximum)))
        self._cols.addWidget(self._value)

        self._slider.valueChanged.connect(self._on_slider_value_changed)

    def get_value(self) -> float:
        return self._slider.value()

    def set_value(self, value: float) -> None:
        self._slider.setValue(value)

    @pyqtSlot()
    def _on_slider_value_changed(self) -> None:
        self._value.setText(f"{self.get_value()}")
