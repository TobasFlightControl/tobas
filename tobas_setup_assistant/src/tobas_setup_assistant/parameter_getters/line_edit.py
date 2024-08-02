from typing import override
from typing import Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QLineEdit

from .base import ParamGetterWidget


class ParamGetterWidget_LineEdit(ParamGetterWidget[str]):
    text_changed = pyqtSignal(str)

    def __init__(self, param_name: str, description_text: Optional[str] = None, default: str = "") -> None:
        super().__init__(param_name, description_text)

        self._line = QLineEdit(default)
        self._rows.addWidget(self._line)

        self._line.textChanged.connect(self._on_text_changed)

    @override
    def get(self) -> str:
        return self._line.text()

    @override
    def set(self, src: str) -> None:
        self._line.setText(src)

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)
