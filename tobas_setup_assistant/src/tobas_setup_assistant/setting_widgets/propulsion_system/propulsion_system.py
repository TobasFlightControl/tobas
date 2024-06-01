from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont

from ..base_setting import BaseSettingWidget
from .available_links import AvailableLinksWidget
from .selected_links import SelectedLinksTabWidget


class PropulsionSystemWidget(BaseSettingWidget):
    NAME = "Propulsion"
    LABEL_PSIZE = 12

    LINK_NAME = "link_name"

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

        self._available = AvailableLinksWidget(main)
        self._available.link_removed.connect(self._on_available_link_removed)
        self._rows.addWidget(self._available)

        self.selected = SelectedLinksTabWidget(main)
        self.selected.link_removed.connect(self._on_selected_link_removed)
        self._rows.addWidget(self.selected)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        self._available.update_internal_data_structures()
        self.selected.update_internal_data_structures()

    @override
    def is_valid(self) -> bool:
        if not self._available.is_valid():
            return False
        if not self.selected.is_valid():
            return False

        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()
        for link_name in self.selected.link_names():
            res[link_name] = self.selected.dump_settings(link_name)
        return res

    @override
    def load_settings(self, data: dict) -> None:
        for link_name, setting in data.items():
            # リンクをAvailableからSelectedに移動させる
            self._available.remove_link(link_name)
            self.selected.add_link(link_name)

            # 選択リンクの設定を更新
            self.selected.load_settings(link_name, setting)

    @pyqtSlot(str)
    def _on_available_link_removed(self, link_name: str) -> None:
        self.selected.add_link(link_name)

    @pyqtSlot(str)
    def _on_selected_link_removed(self, link_name: str) -> None:
        self._available.add_link(link_name)
