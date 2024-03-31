from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import rospy
from abc import abstractmethod
from typing import List, final
from overrides import override
from dynamic_reconfigure import client
from dynamic_reconfigure.msg import ConfigDescription
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget
from tobas_rqt_tools.roslaunch import rosrun
from tobas_rqt_tools.dynamic_reconfigure import get_param_config

from ...common import *

PARAM_DESCRIPTION_TIMEOUT = 3


class BaseController(Widget):
    NAME = TO_DO
    CONTROLLER_PKG = TO_DO
    TAKEOFF_PKG = TO_DO
    LANDING_PKG = TO_DO
    STABLIZE_MODE = TO_DO
    ACROBAT_MODE = TO_DO
    PARAM_SERVER_NODE = TO_DO

    def __init__(self, main: SetupAssistant, abst_text: str) -> None:
        super().__init__(parent=main)
        self._main = main

        self._param_server_process = None
        self._configs: List[dict] = []

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(abst_text)
        self._rows.addWidget(abst)

    @override
    def close(self) -> bool:
        self._param_server_process.terminate()
        return super().close()

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def add_dynamic_params(self) -> None:
        """動的パラメータをウィジェットに反映"""
        raise NotImplementedError()

    @abstractmethod
    def is_applicable(self) -> bool:
        """
        ハードウェアの構造のみから，制御器が適用可能かどうかを返す．

        Returns
        -------
        bool
            制御器が適用可能かどうか．

        Note
        ------
        - 実験データによるモータの設定など，個別の設定方法に依存してはならない．
        """
        raise NotImplementedError()

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def parameter_dict(self) -> dict:
        # 動的パラメータを取得
        cfg: ConfigDescription = rospy.wait_for_message(
            f"/{self.CONTROLLER_PKG}/parameter_descriptions", ConfigDescription, PARAM_DESCRIPTION_TIMEOUT
        )
        dflt = cfg.dflt

        # デフォルトのパラメータを入れる
        res = {self.PARAM_SERVER_NODE: dict()}
        for defaults in [dflt.ints, dflt.doubles, dflt.strs, dflt.bools]:
            for param in defaults:
                res[self.PARAM_SERVER_NODE][param.name] = param.value

        return res

    @final
    def get_dynamic_params(self) -> None:
        self._param_server_process = rosrun(self.CONTROLLER_PKG, "parameter_server_node.py", self.CONTROLLER_PKG)
        cli = client.Client(self.CONTROLLER_PKG, timeout=ROSLAUNCH_TIMEOUT)
        self._configs: List[dict] = cli.get_parameter_descriptions()

    @final
    def _get_param_config(self, name: str) -> dict:
        return get_param_config(self._configs, name)
