from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .commands import Commands


class AddCommandDialog(QDialog):

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)

        self.setWindowTitle("Add Command")

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._cmd_list = QListWidget()
        self._cmd_list.addItems(Commands.values())
        self._cmd_list.setSelectionMode(QListWidget.SingleSelection)  # 単一選択
        rows.addWidget(self._cmd_list)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._ok_button = QPushButton("OK")
        self._ok_button.clicked.connect(self._on_ok_clicked)
        cols.addWidget(self._ok_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.clicked.connect(self.reject)
        cols.addWidget(self._cancel_button)

        self._selected_command = ""

    def selected_command(self) -> str:
        return self._selected_command

    @pyqtSlot()
    def _on_ok_clicked(self) -> None:
        selected_items = self._cmd_list.selectedItems()
        if len(selected_items) == 0:
            return
        elif len(selected_items) == 1:
            self._selected_command = selected_items[0].text()
            self.accept()
        else:
            raise RuntimeError()
