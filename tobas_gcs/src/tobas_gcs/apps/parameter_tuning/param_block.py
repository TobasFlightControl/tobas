from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from dynamic_reconfigure import client
from typing import List, Dict
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.layouts import FormLayout
from tobas_rqt_tools.messages import q_error
from tobas_rqt_tools.utils import place_center
from tobas_tools_py.drone import Drone

from .param_widgets import *


class ParamBlockWidget(QWidget):
    LABEL_PSIZE = 12
    TIMEOUT = 1

    def __init__(self, main: GroundControlStationWidget, drone: Drone, node_name: str, label: str) -> None:
        super().__init__()
        self._main = main
        self._drone = drone
        self._node_name = node_name

        rows = QVBoxLayout()
        self.setLayout(rows)

        label_widget = QLabel(label)
        label_widget.setFont(QFont("Default", self.LABEL_PSIZE, QFont.Bold))
        place_center(label_widget, rows)

        self._form = FormLayout()
        rows.addLayout(self._form)

    def update_internal_data_structures(self) -> None:
        self._form.clear()

    def load(self) -> bool:
        self._form.clear()

        try:
            cli = client.Client(f"/{self._drone.drone_name}/{self._node_name}", timeout=self.TIMEOUT)
        except Exception:
            q_error(self._main, "Failed to connect to dynamic reconfigure server.")
            return False

        configs: Dict = cli.get_configuration(self.TIMEOUT)
        if configs is None:
            q_error(self._main, "Failed to get dynamic parameter configurations.")
            return False

        param_descs: List[dict] = cli.get_parameter_descriptions(self.TIMEOUT)
        if param_descs is None:
            q_error(self._main, "Failed to get dynamic parameter descriptions.")
            return False

        # TODO: QGridLayoutを使うなどして各要素を整列させる (cf. rqt_reconfigure)
        for param_desc in param_descs:
            name = param_desc["name"]
            type_ = param_desc["type"]
            value = configs[name]

            if type_ == "int":
                param_widget = IntParamWidget(param_desc["min"], param_desc["max"])
            elif type_ == "double":
                param_widget = FloatParamWidget(param_desc["min"], param_desc["max"])
            elif type_ == "bool":
                q_error("Configuration for bool parameter is not supported yet.")  # TODO
                return False
            elif type_ == "str":
                q_error("Configuration for str parameter is not supported yet.")  # TODO
                return False
            else:
                q_error(f"Unknown parameter type: {type_}")
                return False

            param_widget.set(value)

            self._form.addRow(QLabel(name), param_widget)

        return True
