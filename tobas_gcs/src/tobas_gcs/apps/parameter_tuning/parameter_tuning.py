from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import os.path as osp
import yaml
from overrides import override
from typing import Dict
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout

from tobas_rqt_tools.layouts import ScrollableVBoxLayout
from tobas_rqt_tools.messages import q_info, q_error, yes_or_no, QMessageLevel
from tobas_tools_py.drone import Drone
from tobas_tools_py.constants import CONTROLLER_NODE_NAME, OBSERVER_NODE_NAME
from tobas_tools_py.package import get_tbs_name, get_tbs_config_name, get_dynamic_params_path

from ...common import CATKIN_WS_TOBAS
from ...utils.ssh_client import SSHClientWrapper
from ..base import BaseAppWidget
from .param_block import ParamBlockWidget, ParamType


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
        self._load_button.clicked.connect(self._on_load_button_clicked)
        cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        self._save_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._save_button.setEnabled(False)
        self._save_button.clicked.connect(self._on_save_button_clicked)
        cols.addWidget(self._save_button)

        self._reset_button = QPushButton("Reset")
        self._reset_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._reset_button.setEnabled(False)
        self._reset_button.clicked.connect(self._on_reset_button_clicked)
        cols.addWidget(self._reset_button)

        cols.addStretch()

        scroll_area = ScrollableVBoxLayout()
        rows.addLayout(scroll_area)

        self._param_blocks = [
            ParamBlockWidget(main, drone, CONTROLLER_NODE_NAME, "Controller"),
            ParamBlockWidget(main, drone, OBSERVER_NODE_NAME, "Observer"),
        ]
        for param_block in self._param_blocks:
            scroll_area.addWidget(param_block)
            param_block.setVisible(False)  # ラベルだけ見えているのは不自然なので最初は隠す

        scroll_area.addStretch()

        self._ssh_client = SSHClientWrapper()

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
            param_block.setVisible(True)  # 読み込みと同時に可視化

        self._save_button.setEnabled(True)
        self._reset_button.setEnabled(True)

        q_info(self._main, "Dynamic parameters are loaded successfully.")

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        # 現在のパラメータが格納された辞書を作成
        try:
            config = self._create_current_config()
        except Exception as e:
            q_error(self._main, str(e))
            return

        # FCに保存
        if not self._save_config_on_fc(config):
            return

        # PCに保存
        if not self._save_config_on_pc(config):
            return

        q_info(self._main, "Dynamic parameters are saved to PC and FC successfully.")

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

    def _create_current_config(self) -> Dict[str, Dict[str, ParamType]]:
        res = {}
        for param_block in self._param_blocks:
            config = param_block.get_current_config()
            param_dict = {}
            for param_desc in param_block.get_parameter_descriptions():
                param_name = param_desc["name"]
                param_dict[param_name] = config[param_name]
            res[param_block.get_node_name()] = param_dict
        return res

    def _save_config_on_fc(self, config: Dict[str, Dict[str, ParamType]]) -> None:
        # SSH接続
        try:
            self._ssh_client.connect()
        except Exception as e:
            q_error(self._main, str(e))
            return False

        # 設定ファイルが存在することを確認
        tbs_path = self._main.pkg_path()  # PC上のTobasパッケージまでのパス
        tbs_name = get_tbs_name(tbs_path)
        config_pkg_name = get_tbs_config_name(tbs_path)
        config_path = osp.join(CATKIN_WS_TOBAS, "src", tbs_name, config_pkg_name, "config", "dynamic_params.yaml")
        if not self._ssh_client.file_exists(config_path):
            q_error(self._main, f"{config_path} does not exist.")
            return False

        # 設定をテキストに変換
        config_text = yaml.safe_dump(config)

        # FCに書き込む
        try:
            self._ssh_client.sftp_write_super(config_path, config_text)
        except Exception as e:
            q_error(self._main, str(e))
            return False

        return True

    def _save_config_on_pc(self, config: Dict[str, Dict[str, ParamType]]) -> bool:
        # 設定ファイルが存在することを確認
        config_path = get_dynamic_params_path(self._main.pkg_path())
        if not osp.exists(config_path):
            q_error(self._main, "Configuration file does not exist on PC.")
            return False

        # PCに書き込む
        with open(config_path, "w") as f:
            yaml.safe_dump(config, f)

        return True
