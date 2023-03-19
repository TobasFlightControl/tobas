from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget
from ..const import *


class ParamGetterWidget_FileDialog(ParamGetterWidget):

    path_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        default: str = "",
        initial_filter: str = "All (*)",
    ) -> None:
        super().__init__(param_name, description_text)
        self._init_filter = initial_filter

        self._options = QFileDialog.Options()
        self._options |= QFileDialog.DontUseNativeDialog

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self.path = QLineEdit(default)
        self.path.setReadOnly(True)
        self.path.setFocusPolicy(Qt.NoFocus)
        self._cols.addWidget(self.path)

        self.browse_button = QPushButton("Browse")
        self._cols.addWidget(self.browse_button)

        self.path.textChanged.connect(self._on_text_changed)
        self.browse_button.clicked.connect(self._on_browse_button_clicked)

    def get(self) -> str:
        return self.line.text()

    def set(self, text: str) -> None:
        self.path.setText(text)

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.path_changed.emit(text)

    @pyqtSlot()
    def _on_browse_button_clicked(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, TITLE, "", self._init_filter, self._options)
        self.path.setText(path)
