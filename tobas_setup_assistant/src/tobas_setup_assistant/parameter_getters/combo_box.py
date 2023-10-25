from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox

from .base import ParamGetterWidget


class ParamGetterWidget_ComboBox(ParamGetterWidget):
    index_changed = pyqtSignal(int)
    text_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        choices: List[str] = [],
        default: str = None,
    ) -> None:
        super().__init__(param_name, description_text)

        self._box = ComboBox()
        self._rows.addWidget(self._box)

        self._box.addItems(choices)

        if default is not None:
            self._box.setCurrentText(default)

        self._box.currentIndexChanged.connect(self._on_index_changed)
        self._box.currentTextChanged.connect(self._on_text_changed)

    def get(self) -> str:
        return self._box.currentText()

    def set(self, text: str) -> None:
        self._box.setCurrentText(text)

    def cur_index(self) -> int:
        return self._box.currentIndex()
    
    def add_items(self, items: List[str])->None:
        self._box.addItems(items)

    @pyqtSlot(int)
    def _on_index_changed(self, idx: int) -> None:
        self.index_changed.emit(idx)

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)
