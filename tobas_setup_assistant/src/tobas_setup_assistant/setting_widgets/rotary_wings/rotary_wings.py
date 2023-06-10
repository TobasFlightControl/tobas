from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from ...parameter_getters import *
from ...constants import *
from ..base_setting import BaseSettingWidget
from .available_links import AvailableLinksWidget
from .selected_links import SelectedLinksWidget


class RotaryWingsWidget(BaseSettingWidget):

    NAME = "Rotary Wings"
    LABEL_PSIZE = 12

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Rotary Wings"
        abst_text = "プロペラの設定を行います．"\
            + "全てのプロペラがX軸前方もしくはZ軸上方に推力を発生することを想定しています．"\
            + "Available Linksからプロペラとして使用するリンクを追加し，"\
            + "それぞれのプロペラに対して必要事項を入力してください．"
        super().__init__(main, title_text, abst_text)

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self.available = AvailableLinksWidget(self._main)
        self._rows.addWidget(self.available)

        self.selected = SelectedLinksWidget(self._main)
        self._rows.addWidget(self.selected)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self.available.define_connections()
        self.selected.define_connections()

    def is_valid(self) -> bool:
        if not self.available.is_valid():
            return False
        if not self.selected.is_valid():
            return False

        return True
