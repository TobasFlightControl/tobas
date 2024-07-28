from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import math
import rospy
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.widgets import FloatSliderTextWidget
from tobas_rqt_tools.layouts import FormLayout
from tobas_rqt_tools.messages import q_error

from tobas_tools_py.drone import Drone
from tobas_gazebo_msgs.srv import (
    GetWindParams,
    GetWindParamsRequest,
    GetWindParamsResponse,
    SetWindParams,
    SetWindParamsRequest,
    SetWindParamsResponse,
)

from ...common import TITLE_PSIZE


class WindParamsWidget(QWidget):
    WAIT_FOR_SERVICE = 30.0  # [s]

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()

        self._main = main
        self._drone = drone

        self._get_sc = None
        self._set_sc = None

        rows = QVBoxLayout()
        self.setLayout(rows)

        title = QLabel("Wind Parameters")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        rows.addWidget(title)

        form = FormLayout()
        rows.addLayout(form)

        self._mean_speed = FloatSliderTextWidget(0.0, 20.0)
        self._mean_speed.value_changed.connect(self._on_value_changed)
        form.addRow(QLabel("Mean Speed [m/s]"), self._mean_speed)

        self._direction = FloatSliderTextWidget(-math.pi, math.pi)
        self._direction.value_changed.connect(self._on_value_changed)
        form.addRow(QLabel("Direction [rad]"), self._direction)

        self._gust_speed_factor = FloatSliderTextWidget(0.0, 10.0)
        self._gust_speed_factor.value_changed.connect(self._on_value_changed)
        form.addRow(QLabel("Gust Speed Factor [-]"), self._gust_speed_factor)

        self._gust_duration = FloatSliderTextWidget(0.0, 10.0)
        self._gust_duration.value_changed.connect(self._on_value_changed)
        form.addRow(QLabel("Gust Duration [s]"), self._gust_duration)

        self._gust_interval = FloatSliderTextWidget(0.0, 30.0)
        self._gust_interval.value_changed.connect(self._on_value_changed)
        form.addRow(QLabel("Gust Interval [s]"), self._gust_interval)

    def initialize(self) -> bool:
        self._get_sc = rospy.ServiceProxy(f"{self._drone.name}/gazebo/get_wind_parameters", GetWindParams)
        self._set_sc = rospy.ServiceProxy(f"{self._drone.name}/gazebo/set_wind_parameters", SetWindParams)

        try:
            self._get_sc.wait_for_service(rospy.Duration(self.WAIT_FOR_SERVICE))
            self._set_sc.wait_for_service(rospy.Duration(self.WAIT_FOR_SERVICE))
        except rospy.ROSException:
            q_error(self._main, "Failed to connect to Gazebo wind server.")
            return False

        # パラメータの初期値を設定
        self._load_current_params()

        return True

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        set_params_req = SetWindParamsRequest()
        set_params_req.params.mean_speed = self._mean_speed.get()
        set_params_req.params.direction = self._direction.get()
        set_params_req.params.gust_speed_factor = self._gust_speed_factor.get()
        set_params_req.params.gust_duration = self._gust_duration.get()
        set_params_req.params.gust_interval = self._gust_interval.get()

        set_params_res: SetWindParamsResponse = self._set_sc.call(set_params_req)
        if not set_params_res.success:
            q_error(self._main, "Failed to set wind parameters.")
            self._load_current_params()
            return

    def _load_current_params(self) -> None:
        get_params_res: GetWindParamsResponse = self._get_sc.call(GetWindParamsRequest())
        cur_params = get_params_res.params

        self._mean_speed.set(cur_params.mean_speed)
        self._direction.set(cur_params.direction)
        self._gust_speed_factor.set(cur_params.gust_speed_factor)
        self._gust_duration.set(cur_params.gust_duration)
        self._gust_interval.set(cur_params.gust_interval)
