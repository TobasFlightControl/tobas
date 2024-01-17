from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from .base import BaseObserver


class CascadeKalmanFilter(BaseObserver):
    NAME = "Cascade Kalman Filter"
    PACKAGE_NAME = "state_estimation_cascade"

    # Dynamic Parameters
    GRAV_VAR = "gravity_variance"

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = (
            "This state estimator is divided into two parts: "
            "an attitude estimator and a position estimator. "
            "It estimates attitude using a complementary filter "
            "based on information from a 6-axis IMU and a geomagnetic sensor. "
            "Then, it estimates the 3D position using a linear Kalman filter, "
            "combining the estimated attitude with information from other sensors."
        )
        super().__init__(main, abst_text)

        gain_acc_description = "Accelerometer gain for the orientation estimation."
        self._gain_acc = ParamGetterWidget_DoubleSpinBox(
            "Acelerometer gain",
            gain_acc_description,
            decimals=3,
            minimum=0.0,
            maximum=1.0,
            default=0.01,
        )
        self._rows.addWidget(self._gain_acc)

        gain_mag_description = "Magnetometer gain for the orientation estimation."
        self._gain_mag = ParamGetterWidget_DoubleSpinBox(
            "Magnetometer gain",
            gain_mag_description,
            decimals=3,
            minimum=0.0,
            maximum=1.0,
            default=0.01,
        )
        self._rows.addWidget(self._gain_mag)

        bias_alpha_description = "Bias estimation gain for the orientation estimation."
        self.bias_alpha = ParamGetterWidget_DoubleSpinBox(
            "Bias estimation gain",
            bias_alpha_description,
            decimals=3,
            minimum=0.0,
            maximum=1.0,
            default=0.01,
        )
        self._rows.addWidget(self.bias_alpha)

        do_bias_estimation_description = (
            "Whether to do bias estimation of the gyroscope readings "
            + "for the orientation estimation."
        )
        self._do_bias_estimation = ParamGetterWidget_CheckBox(
            "Do bias estimation",
            do_bias_estimation_description,
            check_box_text="Do bias estimation",
            default=True,
        )
        self._rows.addWidget(self._do_bias_estimation)

        do_adaptive_gain_description = (
            "Whether to do adaptive gain for the orientation estimation."
        )
        self._do_adaptive_gain = ParamGetterWidget_CheckBox(
            "Do adaptive gain",
            do_adaptive_gain_description,
            check_box_text="Do adaptive gain",
            default=False,
        )
        self._rows.addWidget(self._do_adaptive_gain)

        config = self._get_param_config(self.GRAV_VAR)
        self._grav_var = ParamGetterWidget_SpinBox(
            "Gravity variance",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._grav_var)

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
        res = dict()
        res["orientation_estimator_complement"] = {
            "gain_acc": self._gain_acc.get(),
            "gain_mag": self._gain_mag.get(),
            "bias_alpha": self.bias_alpha.get(),
            "do_bias_estimation": self._do_bias_estimation.get(),
            "do_adaptive_gain": self._do_adaptive_gain.get(),
        }
        res["state_estimator_cascade"] = {
            # Static parameters
            "use_gps": self._main.settings.gps.equipped(),
            self.GPS_HOR_POS_STDDEV_THRESHOLD: self.gps_hor_pos_stddev_threshold.get(),
            self.GPS_VER_POS_STDDEV_THRESHOLD: self.gps_ver_pos_stddev_threshold.get(),
            # Dynamic parameters
            self.GRAV_VAR: self._grav_var.get(),
        }

        return res
