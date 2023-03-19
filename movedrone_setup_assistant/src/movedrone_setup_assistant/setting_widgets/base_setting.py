from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from typing import final
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..const import *


class BaseSettingWidget(QScrollArea):

    ABST_HEIGHT = 100

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
        self._rows.addWidget(abst)

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(lambda: self.setEnabled(True))
        self._main.pkg_generator.generated.connect(lambda: self.setEnabled(False))

    @final
    def _add_dummy_widget(self) -> None:
        """
        余白が空いているとサイズ固定が効かなくなるため，最後に伸縮可能なダミーウィジェットを加える．\\
        ダミーウィジェットを最大まで拡大するようにしておけば，他の要素はなるべく上に詰めてくれる．
        """
        dummy_widget = QWidget()
        dummy_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._rows.addWidget(dummy_widget)
