from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget, QDialog, QPushButton, QVBoxLayout, QHBoxLayout

from tobas_rqt_tools.widgets import ListWidget

from .structs import Commands


class AddCommandDialog(QDialog):
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)

        self.setWindowTitle("Add Command")

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._cmd_list = ListWidget()
        self._cmd_list.addItems(Commands.values())
        self._cmd_list.setSelectionMode(ListWidget.SingleSelection)  # 単一選択
        rows.addWidget(self._cmd_list)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.clicked.connect(self.reject)
        cols.addWidget(self._cancel_button)

        self._ok_button = QPushButton("OK")
        self._ok_button.clicked.connect(self._on_ok_clicked)
        cols.addWidget(self._ok_button)

        self._selected_command = ""

    def selected_command(self) -> str:
        return self._selected_command

    @pyqtSlot()
    def _on_ok_clicked(self) -> None:
        selected_item = self._cmd_list.selected_item()
        if selected_item is None:
            return

        self._selected_command = selected_item.text()
        self.accept()
