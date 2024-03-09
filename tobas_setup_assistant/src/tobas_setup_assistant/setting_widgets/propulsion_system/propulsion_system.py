from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

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
            "Configure the propulsion system. "
            "Please add the link you intend to use for the propulsion system from the Available Links, "
            "and input the necessary information for each."
        )
        super().__init__(main, title_text, abst_text)

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self._available = AvailableLinksWidget(self._main)
        self._rows.addWidget(self._available)

        self.selected = SelectedLinksWidget(self._main)
        self._rows.addWidget(self.selected)

        self._rows.addStretch()

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
