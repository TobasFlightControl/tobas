from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class SimulationWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Gazebo Simulation'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        latitude_0_description = "TODO: instruction"
        self.latitude_0 = ParamGetterWidget_DoubleSpinBox(
            "Latitude of origin [deg]",
            latitude_0_description,
            minimum=-90.,
            maximum=+90.,
            default=35.658099,    # 日本: 北緯35度39分29秒
        )
        self._rows.addWidget(self.latitude_0)

        longitude_0_description = "TODO: instruction"
        self.longitude_0 = ParamGetterWidget_DoubleSpinBox(
            "longitude of origin [deg]",
            longitude_0_description,
            minimum=-180.,
            maximum=+180.,
            default=139.741354,  # 日本: 東経139度44分28秒8759
        )
        self._rows.addWidget(self.longitude_0)

        altitude_0_description = "TODO: instruction"
        self.altitude_0 = ParamGetterWidget_DoubleSpinBox(
            "altitude above sea level [m]",
            altitude_0_description,
            minimum=0.,
            default=24.3900,  # 日本水準原点: https://www.gsi.go.jp/sokuchikijun/suijun-base.html
        )
        self._rows.addWidget(self.altitude_0)

        # 日本経緯度原点に対して国土地理院の地磁気測量値を用いて算出
        # 地磁気測量: https://vldb.gsi.go.jp/sokuchi/geomag/menu_00/index.html

        ref_mag_north_description = "TODO: instruction"
        self.ref_mag_north = ParamGetterWidget_SpinBox(
            "Reference North Magnetic Field Strength [nT]",
            ref_mag_north_description,
            default=30031,
        )
        self._rows.addWidget(self.ref_mag_north)

        ref_mag_east_description = "TODO: instruction"
        self.ref_mag_east = ParamGetterWidget_SpinBox(
            "Reference East Magnetic Field Strength [nT]",
            ref_mag_east_description,
            default=-4116,
        )
        self._rows.addWidget(self.ref_mag_east)

        ref_mag_down_description = "TODO: instruction"
        self.ref_mag_down = ParamGetterWidget_SpinBox(
            "Reference Down Magnetic Field Strength [nT]",
            ref_mag_down_description,
            default=35615,
        )
        self._rows.addWidget(self.ref_mag_down)

        self._add_dummy_widget()
