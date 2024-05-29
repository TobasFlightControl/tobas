from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont

from ..base_setting import BaseSettingWidget
from .signals import PropulsionSystemSignals
from .available_links import AvailableLinksWidget
from .selected_links import SelectedLinksWidget


class PropulsionSystemWidget(BaseSettingWidget):
    NAME = "Propulsion"
    LABEL_PSIZE = 12

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Propulsion System"
        abst_text = (
            "Configure the propulsion system. "
            "Please add the link you intend to use for the propulsion system from the Available Links, "
            "and input the necessary information for each."
        )
        super().__init__(main, title_text, abst_text)

        self._signals = PropulsionSystemSignals()

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self._available = AvailableLinksWidget(self._main, self._signals)
        self._rows.addWidget(self._available)

        self.selected = SelectedLinksWidget(self._main, self._signals)
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
