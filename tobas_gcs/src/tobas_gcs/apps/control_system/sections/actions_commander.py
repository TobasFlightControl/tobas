from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from typing import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QHBoxLayout

from tobas_tools_py.drone import Drone

from ....utils.ssh_client import SSHClientWrapper
from .base_section import BaseControlSystemSectionWidget


class ActionsCommanderWidget(BaseControlSystemSectionWidget):
    LABEL = "Actions"

    BUTTON_WIDTH = 120
    BUTTON_HEIGHT = 50

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._ssh_client = SSHClientWrapper()

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        # TODO: コマンド中にキャンセルできるように

        self._takeoff_button = QPushButton("Takeoff")
        self._takeoff_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._takeoff_button.clicked.connect(self._on_takeoff_button_clicked)
        cols.addWidget(self._takeoff_button)

        self._landing_button = QPushButton("Landing")
        self._landing_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._landing_button.clicked.connect(self._on_landing_button_clicked)
        cols.addWidget(self._landing_button)

        self._rth_button = QPushButton("Return to Home")
        self._rth_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._rth_button.clicked.connect(self._on_rth_button_clicked)
        cols.addWidget(self._rth_button)

        cols.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @pyqtSlot()
    def _on_takeoff_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_landing_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_rth_button_clicked(self) -> None:
        pass  # TODO
