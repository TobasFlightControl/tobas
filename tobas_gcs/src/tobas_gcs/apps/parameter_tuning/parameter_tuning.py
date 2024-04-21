from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.layouts import ScrollableVBoxLayout
from tobas_rqt_tools.messages import q_info, q_error, yes_or_no, QMessageLevel
from tobas_tools_py.drone import Drone

from ...utils.ssh_client import SSHClientWrapper
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
        self._load_button.setEnabled(False)
        cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        self._save_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._save_button.setEnabled(False)
        cols.addWidget(self._save_button)

        self._reset_button = QPushButton("Reset")
        self._reset_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._reset_button.setEnabled(False)
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

        self._ssh_client = SSHClientWrapper()

    @override
    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._save_button.clicked.connect(self._on_save_button_clicked)
        self._reset_button.clicked.connect(self._on_reset_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        for param_block in self._param_blocks:
            param_block.update_internal_data_structures()

        self._load_button.setEnabled(True)
        self._save_button.setEnabled(False)
        self._reset_button.setEnabled(False)

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        for param_block in self._param_blocks:
            if not param_block.load():
                return

        self._save_button.setEnabled(True)
        self._reset_button.setEnabled(True)

        q_info(self._main, "Dynamic parameters are loaded successfully.")

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_reset_button_clicked(self) -> None:
        # 本当に全てのパラメータをリセットしてよいか確認
        if not yes_or_no(
            self._main, "Are you sure you want to reset all parameters to their defaults?", QMessageLevel.WARN
        ):
            return

        # 全てのパラメータをデフォルト値に戻す
        for param_block in self._param_blocks:
            if not param_block.set_to_defaults():
                return

        q_info(self._main, "Dynamic parameters are set to their defaults successfully.")
