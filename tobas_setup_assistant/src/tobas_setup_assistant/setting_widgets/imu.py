from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from .base_setting import BaseSettingWidget
from ..constants import *
from ..parameter_getters import *


class ImuWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Inertial Measurement Unit"
        abst_text = "6軸IMUの設定を行います．データシートを確認し，各値を入力してください．"\
            + "センサフレームは機体フレームに平行であり，値はNWU座標系で得られることを想定しています．"\
            + "Tobasのハードウェアを用いる場合は修正する必要はありません．"
        super().__init__(main, title_text, abst_text)

        link_description = "センサが取り付けられたフレームの名前．"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=100,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        gyro_noise_density_description = ""
        self.gyro_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope noise density (two-sided spectrum)",
            gyro_noise_density_description,
            decimals=9,
            minimum=0.,
            default=3.394e-4,
            suffix=" rad/s/sqrt(Hz)"
        )
        self._rows.addWidget(self.gyro_noise_density)

        gyro_random_walk_description = ""
        self.gyro_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope bias random walk",
            gyro_random_walk_description,
            decimals=9,
            minimum=0.,
            default=3.8785e-5,
            suffix=" rad/s^2/sqrt(Hz)"
        )
        self._rows.addWidget(self.gyro_random_walk)

        gyro_bias_corr_time_description = ""
        self.gyro_bias_corr_time = ParamGetterWidget_SpinBox(
            "Gyroscope bias correlation time constant",
            gyro_bias_corr_time_description,
            minimum=0,
            default=1000,
            suffix=" s"
        )
        self._rows.addWidget(self.gyro_bias_corr_time)

        gyro_turn_on_bias_sigma_description = ""
        self.gyro_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope turn on bias standard deviation",
            gyro_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.,
            default=8.7e-3,
            suffix=" rad/s"
        )
        self._rows.addWidget(self.gyro_turn_on_bias_sigma)

        acc_noise_density_description = ""
        self.acc_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer noise density (two-sided spectrum)",
            acc_noise_density_description,
            decimals=9,
            minimum=0.,
            default=4e-3,
            suffix=" m/s^2/sqrt(Hz)"
        )
        self._rows.addWidget(self.acc_noise_density)

        acc_random_walk_description = ""
        self.acc_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer bias random walk",
            acc_random_walk_description,
            decimals=9,
            minimum=0.,
            default=6e-3,
            suffix=" m/s^3/sqrt(Hz)"
        )
        self._rows.addWidget(self.acc_random_walk)

        acc_bias_corr_time_description = ""
        self.acc_bias_corr_time = ParamGetterWidget_SpinBox(
            "Accelerometer bias correlation time constant",
            acc_bias_corr_time_description,
            minimum=0,
            default=300,
            suffix=" s"
        )
        self._rows.addWidget(self.acc_bias_corr_time)

        acc_turn_on_bias_sigma_description = ""
        self.acc_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer turn on bias standard deviation",
            acc_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.,
            default=0.196,
            suffix=" m/s^2"
        )
        self._rows.addWidget(self.acc_turn_on_bias_sigma)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        return True

    def equipped(self) -> bool:
        return True

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.nwu_fixed_link_names()
        self.link.box.addItems(body_choices)
