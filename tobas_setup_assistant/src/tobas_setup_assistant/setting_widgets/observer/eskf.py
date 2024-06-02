from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtWidgets import QCheckBox, QVBoxLayout

from tobas_rqt_tools.messages import q_error_named

from .base import BaseObserver


class ErrorStateKalmanFilter(BaseObserver):
    NAME = "Error State Kalman Filter"
    PACKAGE_NAME = "state_estimation_eskf"
    ABST_TEXT = (
        "The Error State Kalman Filter (ESKF) is an advanced variant of the Kalman Filter, "
        "tailored for systems with non-linear dynamics. "
        "Unlike the traditional Kalman Filter, which directly estimates the system's state, "
        "the ESKF focuses on estimating the error in the state. "
        "This approach allows for more effective handling of non-linear relationships "
        "between the system state and measurements. "
        "The ESKF operates by linearizing these non-linearities around a nominal state. "
        "It's particularly useful in applications like navigation and tracking, "
        "where precision in estimating orientation and position is crucial, "
        "such as in Inertial Navigation Systems and GPS technology. "
        "The ESKF's blend of accuracy and computational efficiency "
        "makes it a valuable tool in complex engineering tasks."
    )

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        self._do_acc_bias_estimation = QCheckBox(text="Do Accelerometer Bias Estimation")
        self._do_acc_bias_estimation.setChecked(False)
        self._param_rows.addWidget(self._do_acc_bias_estimation)

        self._do_gyro_bias_estimation = QCheckBox(text="Do Gyroscope Bias Estimation")
        self._do_gyro_bias_estimation.setChecked(True)
        self._param_rows.addWidget(self._do_gyro_bias_estimation)

        self._do_gravity_estimation = QCheckBox(text="Do Gravity Estimation")
        self._do_gravity_estimation.setChecked(True)
        self._param_rows.addWidget(self._do_gravity_estimation)

    @override
    def is_valid(self) -> bool:
        # 絶対位置が取得できないとダメ
        no_gps = not self._main.gps.equipped()
        no_odom = not self._main.odometry.equipped()
        if no_gps and no_odom:
            q_error_named(
                self._main, self.NAME, "Absolute position connot be observed. Please review the sensor settings."
            )
            return False

        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._param_rows.count()):
            ckb: QCheckBox = self._param_rows.itemAt(i).widget()
            res[ckb.text()] = ckb.isChecked()
        return res

    @override
    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            ckb: QCheckBox = self._param_rows.itemAt(i).widget()
            ckb.setChecked(data[ckb.text()])

    @override
    def static_parameters(self) -> dict:
        return {
            "use_barometer": False,  # TODO: 選択できるように
            "use_gps": self._main.gps.equipped(),
            "do_acc_bias_estimation": self._do_acc_bias_estimation.isChecked(),
            "do_gyro_bias_estimation": self._do_gyro_bias_estimation.isChecked(),
            "do_gravity_estimation": self._do_gravity_estimation.isChecked(),
            "imu_offset": self._main.imu.offset.get(),
            "barometer_offset": self._main.barometer.offset.get(),
            "gps_offset": self._main.gps.offset.get(),
        }
