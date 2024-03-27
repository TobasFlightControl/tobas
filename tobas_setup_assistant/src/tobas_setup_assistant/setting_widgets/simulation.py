from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import math
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class SimulationWidget(BaseSettingWidget):
    NAME = "Simulation"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Gazebo Simulation"
        abst_text = (
            "Configure the settings for the Gazebo simulation environment. "
            "To enhance the accuracy of the simulation, please input information about the actual operating environment."
        )
        super().__init__(main, title_text, abst_text)

        gravity_description = ""
        self.gravity = ParamGetterWidget_DoubleSpinBox(
            "Gravity",
            gravity_description,
            decimals=6,
            minimum=0.0,
            default=9.80665,  # 標準重力加速度
            suffix=" m/s^2",
        )
        self._rows.addWidget(self.gravity)
        self.gravity.setEnabled(False)  # 重力の変化は無視できるため，標準重力加速度のみを使う

        latitude_0_description = ""
        self.latitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Latitude of origin",
            latitude_0_description,
            decimals=6,
            minimum=-90.0,
            maximum=+90.0,
            default=35.658099,  # 日本: 北緯35度39分29秒
            suffix=" deg",
        )
        self._rows.addWidget(self.latitude_0)

        longitude_0_description = ""
        self.longitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Longitude of origin",
            longitude_0_description,
            decimals=6,
            minimum=-180.0,
            maximum=+180.0,
            default=139.741354,  # 日本: 東経139度44分28秒8759
            suffix=" deg",
        )
        self._rows.addWidget(self.longitude_0)

        altitude_0_description = ""
        self.altitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Altitude above sea level",
            altitude_0_description,
            decimals=3,
            default=24.39,  # 日本水準原点: https://www.gsi.go.jp/sokuchikijun/suijun-base.html
            suffix=" m",
        )
        self._rows.addWidget(self.altitude_0)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()

    @override
    def is_valid(self) -> bool:
        # TODO: 極に近すぎると方角がわからない
        return True
