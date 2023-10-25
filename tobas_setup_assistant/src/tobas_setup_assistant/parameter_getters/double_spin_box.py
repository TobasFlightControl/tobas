from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import DoubleSpinBox

from .base import ParamGetterWidget


class ParamGetterWidget_DoubleSpinBox(ParamGetterWidget):
    value_changed = pyqtSignal(float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        decimals: int = 12,
        minimum: float = -1e9,
        maximum: float = +1e9,
        single_step: float = 1.0,
        default: float = None,
        suffix: str = "",
    ) -> None:
        assert minimum < maximum
        assert single_step > 0.0
        assert decimals >= 0

        super().__init__(param_name, description_text)

        self.spin_box = DoubleSpinBox()
        self._rows.addWidget(self.spin_box)

        self.spin_box.setDecimals(decimals)  # 桁数の設定を最初にしないと，デフォルト値などが潰れてしまう
        self.spin_box.setMinimum(minimum)
        self.spin_box.setMaximum(maximum)
        self.spin_box.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self.spin_box.setValue(default)
        self.spin_box.setSuffix(suffix)

        self.spin_box.valueChanged.connect(self._on_value_changed)

    def get(self) -> float:
        return self.spin_box.value()

    def set(self, value: float) -> None:
        self.spin_box.setValue(value)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(value)
