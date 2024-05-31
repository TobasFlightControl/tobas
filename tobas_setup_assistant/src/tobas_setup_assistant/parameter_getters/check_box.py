from overrides import override
from typing import Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QCheckBox

from .base import ParamGetterWidget


class ParamGetterWidget_CheckBox(ParamGetterWidget):
    toggled = pyqtSignal(bool)

    def __init__(
        self,
        param_name: str,
        description_text: Optional[str] = None,
        check_box_text: Optional[str] = None,
        default: bool = False,
    ) -> None:
        super().__init__(param_name, description_text)

        self._box = QCheckBox(check_box_text)
        self._box.setChecked(default)
        self._box.toggled.connect(self._on_toggled)
        self._rows.addWidget(self._box)

    @override
    def get(self) -> bool:
        return self._box.isChecked()

    @override
    def set(self, src: bool) -> None:
        self._box.setChecked(src)

    @pyqtSlot()
    def _on_toggled(self) -> None:
        self.toggled.emit(self.get())
