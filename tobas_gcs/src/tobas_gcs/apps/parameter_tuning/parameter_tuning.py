from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.layouts import ScrollableVBoxLayout
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget
from .param_block import ParamBlockWidget


class ParameterTuningWidget(BaseAppWidget):
    NAME = "Parameter Tuning"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._load_button = QPushButton("Load")
        self._load_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        self._save_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._save_button)

        self._reset_button = QPushButton("Reset")
        self._reset_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._reset_button)

        cols.addStretch()

        scroll_area = ScrollableVBoxLayout()
        rows.addLayout(scroll_area)

        self._param_blocks = [
            ParamBlockWidget(main, drone, "controller", "Controller"),
            ParamBlockWidget(main, drone, "observer", "Observer"),
        ]
        for param_block in self._param_blocks:
            scroll_area.addWidget(param_block)

        scroll_area.addStretch()

    @override
    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._save_button.clicked.connect(self._on_save_button_clicked)
        self._reset_button.clicked.connect(self._on_reset_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        for param_block in self._param_blocks:
            param_block.update_internal_data_structures()

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        if not self._main.package_loaded():
            q_error(self._main, "Tobas configuration package is not loaded yet.")
            return

        for param_block in self._param_blocks:
            if not param_block.load():
                return

        q_info(self._main, "Dynamic parameters are loaded successfully.")

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_reset_button_clicked(self) -> None:
        pass  # TODO: ダイアログで確認した上で全てのパラメータをデフォルトにする
