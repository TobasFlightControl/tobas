from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from typing import List, final, FrozenSet
import rospy
from dynamic_reconfigure import client
from dynamic_reconfigure.msg import ConfigDescription
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.roslaunch import rosrun
from dh_rqt_tools.dynamic_reconfigure import get_param_config
from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.layouts import FormLayout

from ...common import *

PARAM_DESCRIPTION_TIMEOUT = 3


class BaseController(QWidget):
    NAME = UNKNOWN

    CONTROLLER_PKG = UNKNOWN
    TAKEOFF_PKG = UNKNOWN
    LANDING_PKG = UNKNOWN
    PARAM_SERVER_NODE = UNKNOWN

    COMMAND_MSGS: FrozenSet[str] = frozenset()

    def __init__(self, main: SetupAssistant, abst_text: str) -> None:
        super().__init__()
        self._main = main

        # 動的パラメータの設定を取得
        rosrun(self.CONTROLLER_PKG, "parameter_server_node.py", self.CONTROLLER_PKG)
        cli = client.Client(self.CONTROLLER_PKG, timeout=ROSLAUNCH_TIMEOUT)
        self._configs: List[dict] = cli.get_parameter_descriptions()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(abst_text)
        self._rows.addWidget(abst)

        self.flight_modes = FlightModesWidget()
        self.flight_modes.set_num_modes(DEFAULT_NUM_FLIGHT_MODES, self.COMMAND_MSGS)
        self._rows.addWidget(self.flight_modes)

    @abstractmethod
    def define_connections(self) -> None:
        self._main.signals.num_modes_updated.connect(self._on_num_modes_updated)

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
            f"/{self.CONTROLLER_PKG}/parameter_descriptions",
            ConfigDescription,
            PARAM_DESCRIPTION_TIMEOUT,
        )
        dflt = cfg.dflt

        # デフォルトのパラメータを入れる
        res = {self.PARAM_SERVER_NODE: dict()}
        for defaults in [dflt.ints, dflt.doubles, dflt.strs, dflt.bools]:
            for param in defaults:
                res[self.PARAM_SERVER_NODE][param.name] = param.value

        return res

    @final
    def _get_param_config(self, name: str) -> dict:
        return get_param_config(self._configs, name)

    @final
    @pyqtSlot(int)
    def _on_num_modes_updated(self, num_modes: int) -> None:
        self.flight_modes.set_num_modes(num_modes, self.COMMAND_MSGS)


class FlightModesWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel("Flight Modes")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(label)

        description_text = (
            "プロポの5チャンネルで，プロポから指令するコマンド形式を切り替えることができます．"
            + "各モードに対応するコマンドを選択してください．"
            + "5チャンネルの状態に対する各モードの割当は実機でのRCキャリブレーションで行います．"
        )
        description = Description(description_text)
        self._rows.addWidget(description)

        self._form = FormLayout()
        self._rows.addLayout(self._form)

    def set_num_modes(self, num: int, choices: List[str]) -> None:
        assert num >= 0
        assert len(choices) >= 1

        self._form.clear()

        for i in range(num):
            label = QLabel(f"Flight Mode {i + 1}")
            label.setFont(QFont("Default", pointSize=BODY_PSIZE))

            combo = ComboBox()
            combo.addItems(choices)
            combo.setCurrentIndex(min(i, len(choices) - 1))

            self._form.addRow(label, combo)

    def mode_names(self) -> List[str]:
        res = []
        for i in range(self._form.rowCount()):
            combo: ComboBox = self._form.get_widget(i)
            res.append(combo.currentText())
        return res
