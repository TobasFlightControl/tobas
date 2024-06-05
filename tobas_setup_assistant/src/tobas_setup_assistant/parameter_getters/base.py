from abc import abstractmethod
from typing import Optional, TypeVar, Generic, final
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from ..common import LABEL_PSIZE, Description

T = TypeVar("T")


class ParamGetterWidget(QWidget, Generic[T]):
    """A base class of a widget for getting a user parameter."""

    def __init__(self, param_name: str, description_text: Optional[str] = None) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self._label = QLabel(param_name)
        self._label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        self._label.setAlignment(Qt.AlignTop)
        self._rows.addWidget(self._label)

        if description_text:
            description = Description(description_text)
            self._rows.addWidget(description)

    @abstractmethod
    def get(self) -> T:
        raise NotImplementedError()

    @abstractmethod
    def set(self, src: T) -> None:
        raise NotImplementedError()

    @final
    def name(self) -> str:
        return self._label.text()
