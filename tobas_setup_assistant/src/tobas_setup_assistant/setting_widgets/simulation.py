from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class SimulationWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Gazebo Simulation"
        abst_text = "Gazeboシミュレーション環境の設定を行います．"\
            "シミュレーションの精度を向上させるために，実際の実行環境の情報を入力してください．"
        super().__init__(main, title_text, abst_text)

        gravity_description = ""
        self.gravity = ParamGetterWidget_DoubleSpinBox(
            "Gravity",
            gravity_description,
            decimals=6,
            minimum=0.,
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
            minimum=-90.,
            maximum=+90.,
            default=35.658099,  # 日本: 北緯35度39分29秒
            suffix=" deg",
        )
        self._rows.addWidget(self.latitude_0)

        longitude_0_description = ""
        self.longitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Longitude of origin",
            longitude_0_description,
            decimals=6,
            minimum=-180.,
            maximum=+180.,
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

        # 日本経緯度原点に対して国土地理院の地磁気測量値を用いて算出
        # 地磁気測量: https://vldb.gsi.go.jp/sokuchi/geomag/menu_00/index.html

        ref_mag_north_description = ""
        self.ref_mag_north = ParamGetterWidget_SpinBox(
            "Reference North Magnetic Field Strength",
            ref_mag_north_description,
            default=30031,
            suffix=" nT",
        )
        self._rows.addWidget(self.ref_mag_north)

        ref_mag_east_description = ""
        self.ref_mag_east = ParamGetterWidget_SpinBox(
            "Reference East Magnetic Field Strength",
            ref_mag_east_description,
            default=-4116,
            suffix=" nT",
        )
        self._rows.addWidget(self.ref_mag_east)

        ref_mag_down_description = ""
        self.ref_mag_down = ParamGetterWidget_SpinBox(
            "Reference Down Magnetic Field Strength",
            ref_mag_down_description,
            default=35615,
            suffix=" nT",
        )
        self._rows.addWidget(self.ref_mag_down)

        mean_wind_speed_description = ""
        self.mean_wind_speed = ParamGetterWidget_DoubleSpinBox(
            "Mean wind speed",
            mean_wind_speed_description,
            decimals=1,
            minimum=0.,
            default=0.,
            suffix=" m/s",
        )
        self._rows.addWidget(self.mean_wind_speed)

        const_wind_direction_description = ""
        self.const_wind_direction = ParamGetterWidget_DoubleSpinBox(
            "Constant wind direction (Yaw angle)",
            const_wind_direction_description,
            decimals=2,
            minimum=0.,
            maximum=2 * math.pi,
            default=0.,
            suffix=" m/s",
        )
        self._rows.addWidget(self.const_wind_direction)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()

    def is_valid(self) -> bool:
        return True
