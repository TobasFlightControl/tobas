from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from .base import BaseObserver


class ErrorStateKalmanFilter(BaseObserver):
    NAME = "Error State Kalman Filter"
    PACKAGE_NAME = "state_estimation_eskf"

    # Dynamic Parameters
    GRAV_VAR = "gravity_variance"
    YAW_VAR = "yaw_variance"
    ACC_BIAS_NOISE_VAR_LOG10 = "acc_bias_noise_var_log10"
    GYRO_BIAS_NOISE_VAR_LOG10 = "gyro_bias_noise_var_log10"

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = (
            "An implementation of <a href='https://arxiv.org/abs/1711.02508'>"
            + "Quaternion kinematics for the error-state Kalman filter [Joan Sola, 2017]</a>."
        )
        super().__init__(main, abst_text)

        config = self._get_param_config(self.GRAV_VAR)
        self._grav_var = ParamGetterWidget_SpinBox(
            "Dynamic gravity variance",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" m^2/s^4",
        )
        self._rows.addWidget(self._grav_var)

        config = self._get_param_config(self.YAW_VAR)
        self._yaw_var = ParamGetterWidget_SpinBox(
            "Magnetic yaw angle variance",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" rad^2",
        )
        self._rows.addWidget(self._yaw_var)

        config = self._get_param_config(self.ACC_BIAS_NOISE_VAR_LOG10)
        self._acc_bias_noise_var_log10 = ParamGetterWidget_SpinBox(
            "Accelerometer bias noise variance level",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._acc_bias_noise_var_log10)

        config = self._get_param_config(self.GYRO_BIAS_NOISE_VAR_LOG10)
        self._gyro_bias_noise_var_log10 = ParamGetterWidget_SpinBox(
            "Gyroscope bias noise variance level",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._gyro_bias_noise_var_log10)

    @overrides
    def is_valid(self) -> bool:
        # 絶対位置が取得できないとダメ
        no_gps = not self._main.settings.gps.equipped()
        no_odom = not self._main.settings.odometry.equipped()
        if no_gps and no_odom:
            q_error_named(
                self._main,
                self.NAME,
                "Absolute position connot be observed. Please review the sensor settings.",
            )
            return False

        return True

    @overrides
    def parameter_dict(self) -> dict:
        gps = self._main.settings.gps

        res = dict()
        res["state_estimator_eskf"] = {
            "use_barometer": False,  # TODO: 選択できるように
            "use_gps": gps.equipped(),
            "do_acc_bias_estimation": False,
            "do_gyro_bias_estimation": True,
            "check_covariance_convergence": True,
            self.GPS_HOR_POS_STDDEV_THRESHOLD: self.gps_hor_pos_stddev_threshold.get(),
            self.GPS_VER_POS_STDDEV_THRESHOLD: self.gps_ver_pos_stddev_threshold.get(),
            self.GRAV_VAR: self._grav_var.get(),
            self.YAW_VAR: self._yaw_var.get(),
            self.ACC_BIAS_NOISE_VAR_LOG10: self._acc_bias_noise_var_log10.get(),
            self.GYRO_BIAS_NOISE_VAR_LOG10: self._gyro_bias_noise_var_log10.get(),
        }

        return res
