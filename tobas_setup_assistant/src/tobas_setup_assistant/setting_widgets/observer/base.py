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

from tobas_rqt_tools.roslaunch import rosrun
from tobas_rqt_tools.dynamic_reconfigure import get_param_config

from ...common import *
from ...parameter_getters import *


class BaseObserver(QWidget):
    NAME = UNKNOWN
    PACKAGE_NAME = UNKNOWN

    def __init__(self, main: SetupAssistant, abst_text: str) -> None:
        super().__init__()
        self._main = main

        # 動的パラメータの設定を取得
        rosrun(self.PACKAGE_NAME, "parameter_server_node.py", self.PACKAGE_NAME)
        cli = client.Client(self.PACKAGE_NAME, timeout=ROSLAUNCH_TIMEOUT)
        self._configs: List[dict] = cli.get_parameter_descriptions()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(abst_text)
        self._rows.addWidget(abst)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def parameter_dict(self) -> dict:
        raise NotImplementedError()

    @final
    def _get_param_config(self, name: str) -> dict:
        return get_param_config(self._configs, name)
