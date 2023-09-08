from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from typing import List, final
from dynamic_reconfigure import client
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.roslaunch import rosrun
from dh_rqt_tools.dynamic_reconfigure import get_param_config

from ...common import *


class BaseController(QWidget):
    NAME = "Unknown"

    CONTROLLER_PKG = "Unknown"
    TAKEOFF_PKG = "Unknown"
    LANDING_PKG = "Unknown"

    COMMAND_MSGS = []

    def __init__(self, main: SetupAssistant, abst_text: str) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

        # 動的パラメータをパラメータサーバに登録
        rosrun(self.CONTROLLER_PKG, "parameter_server_node.py", self.CONTROLLER_PKG)
        cli = client.Client(self.CONTROLLER_PKG, timeout=ROSLAUNCH_TIMEOUT)
        self._configs: List[dict] = cli.get_parameter_descriptions()

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
        raise NotImplementedError()

    @final
    def _get_param_config(self, name: str) -> dict:
        return get_param_config(self._configs, name)
