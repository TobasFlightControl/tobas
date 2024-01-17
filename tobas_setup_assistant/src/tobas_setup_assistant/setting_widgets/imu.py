from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import add_spacer

from .base_setting import BaseSettingWidget
from ..common import *
from ..parameter_getters import *


class ImuWidget(BaseSettingWidget):
    NAME = "IMU"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Inertial Measurement Unit"
        abst_text = ""

        super().__init__(main, title_text, abst_text)

        self.offset = ParamGetterWidget_Vector3d(
            "Offset",
            SENSOR_OFFSET_DESCRIPTION,
            suffix=" m",
        )
        self._rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=400,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        gyro_noise_density_description = ""
        self.gyro_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope noise density (two-sided spectrum)",
            gyro_noise_density_description,
            decimals=9,
            minimum=0.0,
            default=0.005,
            suffix=" rad/s/sqrt(Hz)",
        )
        self._rows.addWidget(self.gyro_noise_density)

        gyro_random_walk_description = ""
        self.gyro_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope bias random walk",
            gyro_random_walk_description,
            decimals=9,
            minimum=0.0,
            default=1e-4,
            suffix=" rad/s^2/sqrt(Hz)",
        )
        self._rows.addWidget(self.gyro_random_walk)

        gyro_bias_corr_time_description = ""
        self.gyro_bias_corr_time = ParamGetterWidget_SpinBox(
            "Gyroscope bias correlation time constant",
            gyro_bias_corr_time_description,
            minimum=0,
            default=1000,
            suffix=" s",
        )
        self._rows.addWidget(self.gyro_bias_corr_time)

        gyro_turn_on_bias_sigma_description = ""
        self.gyro_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope turn on bias standard deviation",
            gyro_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.0,
            default=0.05,
            suffix=" rad/s",
        )
        self._rows.addWidget(self.gyro_turn_on_bias_sigma)

        gyro_lpf_cutoff_freq_description = ""
        self.gyro_lpf_cutoff_freq = ParamGetterWidget_SpinBox(
            "Gyroscope LPF cutoff frequency",
            gyro_lpf_cutoff_freq_description,
            minimum=1,
            maximum=400,
            default=20,
            suffix=" Hz",
        )
        self._rows.addWidget(self.gyro_lpf_cutoff_freq)

        acc_noise_density_description = ""
        self.acc_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer noise density (two-sided spectrum)",
            acc_noise_density_description,
            decimals=9,
            minimum=0.0,
            default=0.05,
            suffix=" m/s^2/sqrt(Hz)",
        )
        self._rows.addWidget(self.acc_noise_density)

        acc_random_walk_description = ""
        self.acc_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer bias random walk",
            acc_random_walk_description,
            decimals=9,
            minimum=0.0,
            default=0.01,
            suffix=" m/s^3/sqrt(Hz)",
        )
        self._rows.addWidget(self.acc_random_walk)

        acc_bias_corr_time_description = ""
        self.acc_bias_corr_time = ParamGetterWidget_SpinBox(
            "Accelerometer bias correlation time constant",
            acc_bias_corr_time_description,
            minimum=0,
            default=300,
            suffix=" s",
        )
        self._rows.addWidget(self.acc_bias_corr_time)

        acc_turn_on_bias_sigma_description = ""
        self.acc_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer turn on bias standard deviation",
            acc_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.0,
            default=0.2,
            suffix=" m/s^2",
        )
        self._rows.addWidget(self.acc_turn_on_bias_sigma)

        acc_lpf_cutoff_freq_description = ""
        self.acc_lpf_cutoff_freq = ParamGetterWidget_SpinBox(
            "Accelerometer LPF cutoff frequency",
            acc_lpf_cutoff_freq_description,
            minimum=1,
            maximum=400,
            default=20,
            suffix=" Hz",
        )
        self._rows.addWidget(self.acc_lpf_cutoff_freq)

        mag_gauss_noise_description = ""
        self.mag_gauss_noise = ParamGetterWidget_SpinBox(
            "Magnetometer standard deviation of additive white gaussian noise",
            mag_gauss_noise_description,
            minimum=0,
            default=80,
            suffix=" nT",
        )
        self._rows.addWidget(self.mag_gauss_noise)

        mag_uniform_noise_description = ""
        self.mag_uniform_noise = ParamGetterWidget_SpinBox(
            "Magnetometer symmetric bounds of uniform noise for initial sensor bias",
            mag_uniform_noise_description,
            minimum=0,
            default=400,
            suffix=" nT",
        )
        self._rows.addWidget(self.mag_uniform_noise)

        add_spacer(self._rows)

    @overrides
    def define_connections(self) -> None:
        super().define_connections()

    @overrides
    def is_valid(self) -> bool:
        return True

    def equipped(self) -> bool:
        return True
