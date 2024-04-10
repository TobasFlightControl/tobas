from typing import Optional
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_std_tools_py.string import convert_superscript
from tobas_rqt_tools.widgets import DoubleSpinBox

from .base import ParamGetterWidget


class ParamGetterWidget_DoubleSpinBox(ParamGetterWidget):
    value_changed = pyqtSignal(float)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
        decimals: int = 12,
        minimum: float = -1e9,
        maximum: float = +1e9,
        single_step: float = 1.0,
        default: Optional[float] = None,
        suffix: str = "",
    ) -> None:
        assert minimum < maximum
        assert single_step > 0.0
        assert decimals >= 0

        super().__init__(param_name, description_text)

        self._spin_box = DoubleSpinBox()
        self._rows.addWidget(self._spin_box)

        self._spin_box.setDecimals(decimals)  # 桁数の設定を最初にしないと，デフォルト値などが潰れてしまう
        self._spin_box.setMinimum(minimum)
        self._spin_box.setMaximum(maximum)
        self._spin_box.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self._spin_box.setValue(default)
        self._spin_box.setSuffix(convert_superscript(suffix))
        self._spin_box.setFocusPolicy(Qt.StrongFocus)
        self._spin_box.valueChanged.connect(self._on_value_changed)

    def get(self) -> float:
        return self._spin_box.value()

    def set(self, value: float) -> None:
        self._spin_box.setValue(value)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(value)
