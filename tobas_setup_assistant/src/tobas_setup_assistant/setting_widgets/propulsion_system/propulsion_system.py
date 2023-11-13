from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from ...parameter_getters import *
from ...common import *
from ..base_setting import BaseSettingWidget
from .available_links import AvailableLinksWidget
from .selected_links import SelectedLinksWidget


class PropulsionSystemWidget(BaseSettingWidget):
    NAME = "Propulsion"
    LABEL_PSIZE = 12

    add_link = pyqtSignal(str)  # selectedにリンクを追加
    remove_link = pyqtSignal(str)  # selectedからリンクを削除

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Propulsion System"
        abst_text = (
            "推進系の設定を行います．"
            + "Available Linksから推進系として使用するリンクを追加し，"
            + "それぞれに対して必要事項を入力してください．"
        )
        super().__init__(main, title_text, abst_text)

        links_label = QLabel("Available Links")
        links_label.setFont(
            QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold)
        )
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self._available = AvailableLinksWidget(self._main)
        self._rows.addWidget(self._available)

        self.selected = SelectedLinksWidget(self._main)
        self._rows.addWidget(self.selected)

        add_expanding_widget(self._rows)

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._available.define_connections()
        self.selected.define_connections()

    @overrides
    def is_valid(self) -> bool:
        if not self._available.is_valid():
            return False
        if not self.selected.is_valid():
            return False

        return True
