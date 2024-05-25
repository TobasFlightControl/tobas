from PyQt5.QtCore import pyqtSignal, pyqtSlot

from tobas_rqt_tools.widgets import SpinBox, DoubleSpinBox


class IntParam(SpinBox):
    value_changed = pyqtSignal(str, int)

    def __init__(self, name: str):
        super().__init__()
        self._name = name
        self.valueChanged.connect(self._on_value_changed)

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(self._name, value)


class FloatParam(DoubleSpinBox):
    value_changed = pyqtSignal(str, float)

    def __init__(self, name: str):
        super().__init__()
        self._name = name
        self.valueChanged.connect(self._on_value_changed)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self._name, value)
