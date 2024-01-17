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

    GPS_HOR_POS_STDDEV_THRESHOLD = "gps_horizontal_position_stddev_threshold"
    GPS_VER_POS_STDDEV_THRESHOLD = "gps_vertical_position_stddev_threshold"

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

        gps_hor_pos_stddev_threshold_description = (
            "Threshold for the standard deviation relative to the true value of the horizontal position "
            "when aligning the initial position using GPS. "
            "A smaller value results in a more accurate initial position determination, "
            "but increases the time taken for alignment."
        )
        self.gps_hor_pos_stddev_threshold = ParamGetterWidget_DoubleSpinBox(
            "GPS horizontal position std. dev threshold",
            gps_hor_pos_stddev_threshold_description,
            decimals=2,
            minimum=0.01,
            maximum=1.0,
            default=0.3,
            suffix=" m",
        )
        self._rows.addWidget(self.gps_hor_pos_stddev_threshold)

        gps_ver_pos_stddev_threshold_description = (
            "Threshold for the standard deviation relative to the true value of the vertical position "
            "when aligning the initial position using GPS. "
            "A smaller value results in a more accurate initial position determination, "
            "but increases the time taken for alignment."
        )
        self.gps_ver_pos_stddev_threshold = ParamGetterWidget_DoubleSpinBox(
            "GPS vertical position std. dev threshold",
            gps_ver_pos_stddev_threshold_description,
            decimals=2,
            minimum=0.01,
            maximum=1.0,
            default=0.6,
            suffix=" m",
        )
        self._rows.addWidget(self.gps_ver_pos_stddev_threshold)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def parameter_dict(self) -> dict:
        raise NotImplementedError()

    @final
    def _get_param_config(self, name: str) -> dict:
        return get_param_config(self._configs, name)
