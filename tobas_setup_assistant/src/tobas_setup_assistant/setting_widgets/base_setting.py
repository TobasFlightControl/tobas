from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..common import *


class BaseSettingWidget(QScrollArea):

    ABST_HEIGHT = 100

    NAME = "Unknown"

    def __init__(self, main: SetupAssistant, title_text: str, abst_text: str) -> None:
        super().__init__()
        self._main = main

        self.setWidgetResizable(True)  # この設定が必須．無いとオブジェクトが潰れてしまう．
        self.setEnabled(False)  # 基本的にモデルが読み込まれて初めてアクティブになる

        # QScrollAreaを使う際は，QLayoutの前にQWidgetを挟む必要がある．
        inner = QWidget()
        self.setWidget(inner)
        self._rows = QVBoxLayout()
        inner.setLayout(self._rows)

        title = QLabel(title_text)
        title.setFont(QFont('Default', pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setFixedHeight(self.ABST_HEIGHT)
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

    @abstractmethod
    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(lambda: self.setEnabled(True))
        self._main.pkg_generator.generated.connect(lambda: self.setEnabled(False))

    @abstractmethod
    def is_valid(self) -> bool:
        """ Returns true if user configuration is valid. """
        raise NotImplementedError()
