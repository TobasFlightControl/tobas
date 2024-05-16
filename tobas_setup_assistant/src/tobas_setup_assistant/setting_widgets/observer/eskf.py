from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from tobas_rqt_tools.messages import q_error_named

from .base import BaseObserver


class ErrorStateKalmanFilter(BaseObserver):
    NAME = "Error State Kalman Filter"
    PACKAGE_NAME = "state_estimation_eskf"

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = (
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
        super().__init__(main, abst_text)

    @override
    def is_valid(self) -> bool:
        # 絶対位置が取得できないとダメ
        no_gps = not self._main.settings.gps.equipped()
        no_odom = not self._main.settings.odometry.equipped()
        if no_gps and no_odom:
            q_error_named(
                self._main, self.NAME, "Absolute position connot be observed. Please review the sensor settings."
            )
            return False

        return True

    @override
    def static_parameters(self) -> dict:
        return {
            "use_barometer": False,  # TODO: 選択できるように
            "use_gps": self._main.settings.gps.equipped(),
            "do_acc_bias_estimation": False,
            "do_gyro_bias_estimation": True,
            "do_gravity_estimation": True,
            "imu_offset": self._main.settings.imu.offset.get(),
            "barometer_offset": self._main.settings.barometer.offset.get(),
            "gps_offset": self._main.settings.gps.offset.get(),
        }
