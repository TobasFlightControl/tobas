from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..constants import *


class ParamGetterWidget(QWidget):

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
    ) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel(param_name)
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        self._rows.addWidget(label)

        if description_text:
            description = QLabel(description_text)
            description.setFont(QFont("Default", pointSize=BODY_PSIZE))
            description.setAlignment(Qt.AlignTop)
            description.setWordWrap(True)
            description.setOpenExternalLinks(True)
            self._rows.addWidget(description)

    @abstractmethod
    def get(self):
        raise NotImplementedError

    @abstractmethod
    def set(self):
        raise NotImplementedError
