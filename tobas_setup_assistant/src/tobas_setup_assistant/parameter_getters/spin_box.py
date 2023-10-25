from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import SpinBox

from .base import ParamGetterWidget


class ParamGetterWidget_SpinBox(ParamGetterWidget):
    value_changed = pyqtSignal(int)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        minimum: int = -(10**9),
        maximum: int = +(10**9),
        single_step: int = 1,
        default: int = None,
        suffix: str = "",
    ) -> None:
        assert minimum < maximum
        assert single_step > 0

        super().__init__(param_name, description_text)

        self.spin_box = SpinBox()
        self._rows.addWidget(self.spin_box)

        self.spin_box.setMinimum(minimum)
        self.spin_box.setMaximum(maximum)
        self.spin_box.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self.spin_box.setValue(default)
        self.spin_box.setSuffix(suffix)

        self.spin_box.setFocusPolicy(Qt.StrongFocus)

        self.spin_box.valueChanged.connect(self._on_value_changed)

    def get(self) -> int:
        return self.spin_box.value()

    def set(self, value: int) -> None:
        self.spin_box.setValue(value)

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(value)
