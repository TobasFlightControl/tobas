from abc import abstractmethod
from typing import Optional
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from ..common import LABEL_PSIZE, Description


class ParamGetterWidget(QWidget):
    def __init__(self, param_name: str, description_text: Optional[str] = None) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel(param_name)
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        self._rows.addWidget(label)

        if description_text:
            description = Description(description_text)
            self._rows.addWidget(description)

    @abstractmethod
    def get(self):
        raise NotImplementedError()

    @abstractmethod
    def set(self):
        raise NotImplementedError()
