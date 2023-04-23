from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget


class ParamGetterWidget_CheckBox(ParamGetterWidget):

    toggled = pyqtSignal(bool)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        check_box_text: str = None,
        default: bool = False,
    ) -> None:
        super().__init__(param_name, description_text)

        self.box = QCheckBox(check_box_text)
        self.box.setChecked(default)
        self.box.toggled.connect(self._on_toggled)
        self._rows.addWidget(self.box)

    def get(self) -> bool:
        return self.box.isChecked()

    def set(self, is_checked: bool) -> None:
        self.box.setChecked(is_checked)

    @pyqtSlot()
    def _on_toggled(self) -> None:
        self.toggled.emit(self.get())
