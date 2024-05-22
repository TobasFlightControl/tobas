from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from dynamic_reconfigure import client
from typing import List, Dict, TypeVar
from functools import partial
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.layouts import FormLayout
from tobas_rqt_tools.messages import q_error
from tobas_rqt_tools.utils import place_center
from tobas_tools_py.drone import Drone

from ...common import WAIT_FOR_SERVER
from .param_widgets import IntParamWidget, FloatParamWidget

ParamType = TypeVar("ParamType", int, float, bool, str)


class ParamBlockWidget(QWidget):
    LABEL_PSIZE = 12

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

        self._client = None

    def update_internal_data_structures(self) -> None:
        self._form.clear()

        if self._client is not None:
            self._client.close()
            self._client = None

    def load(self) -> bool:
        self._form.clear()

        # Dynamic Reconfigureのクライアントを作成
        if self._client is None:
            try:
                self._client = client.Client(f"{self._drone.drone_name}/{self._node_name}", timeout=WAIT_FOR_SERVER)
            except Exception:
                q_error(self._main, "Failed to connect to dynamic reconfigure server.")
                return False

        try:
            config = self.get_current_config()
        except Exception as e:
            q_error(self._main, str(e))
            return False

        try:
            param_descriptions = self.get_parameter_descriptions()
        except Exception as e:
            q_error(self._main, str(e))
            return False

        # TODO: QGridLayoutを使うなどして各要素を整列させる (cf. rqt_reconfigure)
        for param_desc in param_descriptions:
            name: str = param_desc["name"]
            type_: str = param_desc["type"]
            value: ParamType = config[name]

            if type_ == "int":
                param_widget = IntParamWidget(param_desc["min"], param_desc["max"])
                param_widget.value_changed.connect(partial(self._on_int_param_changed, name=name))
            elif type_ == "double":
                param_widget = FloatParamWidget(param_desc["min"], param_desc["max"])
                param_widget.value_changed.connect(partial(self._on_float_param_changed, name=name))
            elif type_ == "bool":
                q_error(self._main, "Configuration for bool parameter is not supported yet.")  # TODO
                return False
            elif type_ == "str":
                q_error(self._main, "Configuration for str parameter is not supported yet.")  # TODO
                return False
            else:
                q_error(self._main, f"Unknown parameter type: {type_}")
                return False

            param_widget.set(value)

            self._form.addRow(QLabel(name), param_widget)

        return True

    def set_to_defaults(self) -> bool:
        try:
            param_descriptions = self.get_parameter_descriptions()
        except Exception as e:
            q_error(self._main, str(e))
            return False

        config = {}
        for param_desc in param_descriptions:
            config[param_desc["name"]] = param_desc["default"]

        self._client.update_configuration(config)
        return self.load()

    def get_node_name(self) -> str:
        return self._node_name

    def get_parameter_descriptions(self) -> List[dict]:
        res = self._client.get_parameter_descriptions(timeout=WAIT_FOR_SERVER)
        if res is None:
            raise RuntimeError("Failed to get parameter descriptions.")
        return res

    def get_current_config(self) -> Dict[str, ParamType]:
        """
        現在のパラメータを取得する．

        NOTE
        ----------
        - 辞書にはgroupsなどの不要な値も含まれ，そのままファイルに変換することはできない．
        """
        res = self._client.get_configuration(timeout=WAIT_FOR_SERVER)
        if res is None:
            raise RuntimeError("Failed to get current configurations.")
        return res

    @pyqtSlot(int)
    def _on_int_param_changed(self, value: int, name: str) -> None:
        self._client.update_configuration({name: value})

    @pyqtSlot(float)
    def _on_float_param_changed(self, value: float, name: str) -> None:
        self._client.update_configuration({name: value})

    @pyqtSlot(bool)
    def _on_bool_param_changed(self, value: bool, name: str) -> None:
        self._client.update_configuration({name: value})

    @pyqtSlot(str)
    def _on_str_param_changed(self, value: str, name: str) -> None:
        self._client.update_configuration({name: value})
