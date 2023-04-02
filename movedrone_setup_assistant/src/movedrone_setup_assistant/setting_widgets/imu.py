from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..const import *
from ..parameter_getters import *


class ImuWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Inertial Measurement Unit'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        self.use_custom_imu = QCheckBox("Use custom IMU")
        self.use_custom_imu.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.use_custom_imu)

        gyro_noise_density_description = "TODO: instruction"
        self.gyro_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope noise density (two-sided spectrum)",
            gyro_noise_density_description,
            decimals=9,
            minimum=0.,
            default=3.394e-4,
            suffix=" rad/s/sqrt(Hz)"
        )
        self._rows.addWidget(self.gyro_noise_density)

        gyro_random_walk_description = "TODO: instruction"
        self.gyro_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope bias random walk",
            gyro_random_walk_description,
            decimals=9,
            minimum=0.,
            default=3.8785e-5,
            suffix=" rad/s^2/sqrt(Hz)"
        )
        self._rows.addWidget(self.gyro_random_walk)

        gyro_bias_corr_time_description = "TODO: instruction"
        self.gyro_bias_corr_time = ParamGetterWidget_SpinBox(
            "Gyroscope bias correlation time constant",
            gyro_bias_corr_time_description,
            minimum=0,
            default=1000,
            suffix=" s"
        )
        self._rows.addWidget(self.gyro_bias_corr_time)

        gyro_turn_on_bias_sigma_description = "TODO: instruction"
        self.gyro_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope turn on bias standard deviation",
            gyro_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.,
            default=8.7e-3,
            suffix=" rad/s"
        )
        self._rows.addWidget(self.gyro_turn_on_bias_sigma)

        acc_noise_density_description = "TODO: instruction"
        self.acc_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer noise density (two-sided spectrum)",
            acc_noise_density_description,
            decimals=9,
            minimum=0.,
            default=4e-3,
            suffix=" m/s^2/sqrt(Hz)"
        )
        self._rows.addWidget(self.acc_noise_density)

        acc_random_walk_description = "TODO: instruction"
        self.acc_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer bias random walk",
            acc_random_walk_description,
            decimals=9,
            minimum=0.,
            default=6e-3,
            suffix=" m/s^3/sqrt(Hz)"
        )
        self._rows.addWidget(self.acc_random_walk)

        acc_bias_corr_time_description = "TODO: instruction"
        self.acc_bias_corr_time = ParamGetterWidget_SpinBox(
            "Accelerometer bias correlation time constant",
            acc_bias_corr_time_description,
            minimum=0,
            default=300,
            suffix=" s"
        )
        self._rows.addWidget(self.acc_bias_corr_time)

        acc_turn_on_bias_sigma_description = "TODO: instruction"
        self.acc_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer turn on bias standard deviation",
            acc_turn_on_bias_sigma_description,
            decimals=9,
            minimum=0.,
            default=0.196,
            suffix=" m/s^2"
        )
        self._rows.addWidget(self.acc_turn_on_bias_sigma)

        self._add_dummy_widget()
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.use_custom_imu.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.link.box.addItems(body_choices)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.use_custom_imu.isChecked():
            self.gyro_noise_density.setVisible(True)
            self.gyro_random_walk.setVisible(True)
            self.gyro_bias_corr_time.setVisible(True)
            self.gyro_turn_on_bias_sigma.setVisible(True)
            self.acc_noise_density.setVisible(True)
            self.acc_random_walk.setVisible(True)
            self.acc_bias_corr_time.setVisible(True)
            self.acc_turn_on_bias_sigma.setVisible(True)
        else:
            self.gyro_noise_density.setVisible(False)
            self.gyro_random_walk.setVisible(False)
            self.gyro_bias_corr_time.setVisible(False)
            self.gyro_turn_on_bias_sigma.setVisible(False)
            self.acc_noise_density.setVisible(False)
            self.acc_random_walk.setVisible(False)
            self.acc_bias_corr_time.setVisible(False)
            self.acc_turn_on_bias_sigma.setVisible(False)
