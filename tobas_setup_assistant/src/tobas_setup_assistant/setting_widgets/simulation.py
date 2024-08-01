from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant
    from ..parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from ..parameter_getters import (
    ParamGetterWidget_SpinBox,
    ParamGetterWidget_DoubleSpinBox,
)
from .base_setting import BaseSettingWidget


class SimulationWidget(BaseSettingWidget):
    NAME = "Simulation"
    TITLE_TEXT = "Define Simulation Environment"
    ABST_TEXT = (
        "Configure the settings for the Gazebo simulation environment. "
        "To enhance the accuracy of the simulation, please input information about the actual operating environment."
    )

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        gravity_description = ""
        self.gravity = ParamGetterWidget_DoubleSpinBox(
            "Gravity",
            gravity_description,
            decimals=6,
            minimum=0.0,
            default=9.80665,
            suffix=" m/s^2",  # 標準重力加速度
        )
        self._param_rows.addWidget(self.gravity)
        self.gravity.setEnabled(False)  # 重力の変化は無視できるため，標準重力加速度のみを使う

        latitude_0_description = ""
        self.latitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Latitude of Origin",
            latitude_0_description,
            decimals=6,
            minimum=-90.0,
            maximum=+90.0,
            default=35.658099,  # 日本: 北緯35度39分29秒
            suffix=" deg",
        )
        self._param_rows.addWidget(self.latitude_0)

        longitude_0_description = ""
        self.longitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Longitude of Origin",
            longitude_0_description,
            decimals=6,
            minimum=-180.0,
            maximum=+180.0,
            default=139.741354,  # 日本: 東経139度44分28秒8759
            suffix=" deg",
        )
        self._param_rows.addWidget(self.longitude_0)

        altitude_0_description = ""
        self.altitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Altitude Above Sea Level",
            altitude_0_description,
            decimals=3,
            default=24.39,  # 日本水準原点: https://www.gsi.go.jp/sokuchikijun/suijun-base.html
            suffix=" m",
        )
        self._param_rows.addWidget(self.altitude_0)

        max_model_error_rate_description = ""
        self.max_model_error_rate = ParamGetterWidget_SpinBox(
            "Max Model Error Rate",
            max_model_error_rate_description,
            minimum=0,
            maximum=1000,
            default=10,
            suffix=" %",
        )
        self._param_rows.addWidget(self.max_model_error_rate)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        # TODO: 極に近すぎると方角がわからない
        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()
        return res

    @override
    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])
