from typing import override
from typing import Optional
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from tobas_std_tools_py.string import convert_superscript
from tobas_rqt_tools.widgets import SpinBox

from .base import ParamGetterWidget


class ParamGetterWidget_SpinBox(ParamGetterWidget[int]):
    value_changed = pyqtSignal(int)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
        minimum: int = -(1 << 31),
        maximum: int = (1 << 31) - 1,
        single_step: int = 1,
        default: Optional[int] = None,
        suffix: str = "",
    ) -> None:
        assert minimum < maximum
        assert single_step > 0

        super().__init__(param_name, description_text)

        self._spin_box = SpinBox()
        self._rows.addWidget(self._spin_box)

        self._spin_box.setMinimum(minimum)
        self._spin_box.setMaximum(maximum)
        self._spin_box.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self._spin_box.setValue(default)
        self._spin_box.setSuffix(convert_superscript(suffix))
        self._spin_box.setFocusPolicy(Qt.StrongFocus)
        self._spin_box.valueChanged.connect(self._on_value_changed)

    @override
    def get(self) -> int:
        return self._spin_box.value()

    @override
    def set(self, src: int) -> None:
        self._spin_box.setValue(src)

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(value)
