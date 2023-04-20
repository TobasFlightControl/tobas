from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox

from .base import ParamGetterWidget


class ParamGetterWidget_ComboBox(ParamGetterWidget):

    text_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        choices: List[str] = [],
        default: str = None,
    ) -> None:
        super().__init__(param_name, description_text)

        self.box = ComboBox()
        self._rows.addWidget(self.box)

        self.box.addItems(choices)

        if default is not None:
            self.box.setCurrentText(default)

        self.box.currentTextChanged.connect(self._on_text_changed)

    def get(self) -> str:
        return self.box.currentText()
    
    def set(self, text: str) -> None:
        self.box.setCurrentText(text)

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)
