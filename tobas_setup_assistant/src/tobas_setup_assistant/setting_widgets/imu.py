from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant
    from ..parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from ..common import SENSOR_OFFSET_DESCRIPTION
from ..parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_Vector3d, ParamGetterWidget_DoubleSpinBox
from .base_setting import BaseSettingWidget


class ImuWidget(BaseSettingWidget):
    NAME = "IMU"
    TITLE_TEXT = "Define Inertial Measurement Unit"
    ABST_TEXT = ""  # TODO

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        self.offset = ParamGetterWidget_Vector3d("Offset", SENSOR_OFFSET_DESCRIPTION, suffix=" m")
        self._param_rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate", update_rate_description, minimum=1, default=400, suffix=" Hz"
        )
        self._param_rows.addWidget(self.update_rate)

        gyro_noise_density_description = ""
        self.gyro_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope noise density (two-sided spectrum)",
            gyro_noise_density_description,
            decimals=9,
            minimum=0.0,
            default=0.005,
            suffix=" rad/s/sqrt(Hz)",
        )
        self._param_rows.addWidget(self.gyro_noise_density)

        gyro_random_walk_description = ""
        self.gyro_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope bias random walk",
            gyro_random_walk_description,
            decimals=9,
            minimum=0.0,
            default=1e-4,
            suffix=" rad/s^2/sqrt(Hz)",
        )
        self._param_rows.addWidget(self.gyro_random_walk)

        gyro_bias_corr_time_description = ""
        self.gyro_bias_corr_time = ParamGetterWidget_SpinBox(
            "Gyroscope bias correlation time constant",
            gyro_bias_corr_time_description,
            minimum=0,
            default=1000,
            suffix=" s",
        )
        self._param_rows.addWidget(self.gyro_bias_corr_time)

        gyro_turn_on_bias_sigma_description = ""
        self.gyro_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope turn on bias standard deviation",
            gyro_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.0,
            default=0.05,
            suffix=" rad/s",
        )
        self._param_rows.addWidget(self.gyro_turn_on_bias_sigma)

        gyro_lpf_cutoff_freq_description = ""
        self.gyro_lpf_cutoff_freq = ParamGetterWidget_SpinBox(
            "Gyroscope LPF cutoff frequency",
            gyro_lpf_cutoff_freq_description,
            minimum=1,
            maximum=400,
            default=20,
            suffix=" Hz",
        )
        self._param_rows.addWidget(self.gyro_lpf_cutoff_freq)

        acc_noise_density_description = ""
        self.acc_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer noise density (two-sided spectrum)",
            acc_noise_density_description,
            decimals=9,
            minimum=0.0,
            default=0.05,
            suffix=" m/s^2/sqrt(Hz)",
        )
        self._param_rows.addWidget(self.acc_noise_density)

        acc_random_walk_description = ""
        self.acc_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer bias random walk",
            acc_random_walk_description,
            decimals=9,
            minimum=0.0,
            default=0.01,
            suffix=" m/s^3/sqrt(Hz)",
        )
        self._param_rows.addWidget(self.acc_random_walk)

        acc_bias_corr_time_description = ""
        self.acc_bias_corr_time = ParamGetterWidget_SpinBox(
            "Accelerometer bias correlation time constant",
            acc_bias_corr_time_description,
            minimum=0,
            default=300,
            suffix=" s",
        )
        self._param_rows.addWidget(self.acc_bias_corr_time)

        acc_turn_on_bias_sigma_description = ""
        self.acc_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer turn on bias standard deviation",
            acc_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.0,
            default=0.2,
            suffix=" m/s^2",
        )
        self._param_rows.addWidget(self.acc_turn_on_bias_sigma)

        acc_lpf_cutoff_freq_description = ""
        self.acc_lpf_cutoff_freq = ParamGetterWidget_SpinBox(
            "Accelerometer LPF cutoff frequency",
            acc_lpf_cutoff_freq_description,
            minimum=1,
            maximum=400,
            default=20,
            suffix=" Hz",
        )
        self._param_rows.addWidget(self.acc_lpf_cutoff_freq)

        mag_gauss_noise_description = ""
        self.mag_gauss_noise = ParamGetterWidget_SpinBox(
            "Magnetometer standard deviation of additive white gaussian noise",
            mag_gauss_noise_description,
            minimum=0,
            default=80,
            suffix=" nT",
        )
        self._param_rows.addWidget(self.mag_gauss_noise)

        mag_uniform_noise_description = ""
        self.mag_uniform_noise = ParamGetterWidget_SpinBox(
            "Magnetometer symmetric bounds of uniform noise for initial sensor bias",
            mag_uniform_noise_description,
            minimum=0,
            default=400,
            suffix=" nT",
        )
        self._param_rows.addWidget(self.mag_uniform_noise)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
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

    def equipped(self) -> bool:
        return True
