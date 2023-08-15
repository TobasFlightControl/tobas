from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.messages import q_error_named

from tobas_msgs.msg import RollPitchYawThrust, RollPitchYawrateThrust, VelocityYaw, PositionYaw

from ...parameter_getters import *
from ...common import *
from .base import BaseController


class MultirotorLMPC(BaseController):

    NAME = "Linear Model Predictive Control"

    CONTROLLER_PKG = "tobas_multirotor_controller"
    TAKEOFF_PKG = "tobas_multirotor_takeoff"
    LANDING_PKG = "tobas_multirotor_landing"

    COMMAND_MSGS = [
        RollPitchYawThrust,
        RollPitchYawrateThrust,
        VelocityYaw,
        PositionYaw,
    ]

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "位置制御にPD制御，姿勢制御に線形モデル予測制御を用いた制御器です．"
        super().__init__(main, abst_text)

        hor_natural_freq_description = "PD制御における水平方向の自然周波数．"\
            + "大きいほど応答速度が速くなりますが，"\
            + "大きすぎると遅延やモデル化誤差などの要因により振動が発生する恐れがあります．"
        self.hor_natural_freq = ParamGetterWidget_DoubleSpinBox(
            "Horizontal natural frequency (Translation controller)",
            hor_natural_freq_description,
            decimals=2,
            minimum=0.1,
            default=1.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.hor_natural_freq)

        hor_damp_ratio_description = "PD制御における水平方向の減衰比．"\
            + "大きいほどオーバーシュートが小さくなりますが，"\
            + "大きすぎると速度変化に過剰に反応して振動が発生する恐れがあります．"\
            + "逆に小さいほど応答速度が速くなりますが，"\
            + "小さすぎるとオーバーシュートが大きくなり目標位置付近での振動が大きくなります．"\
            + "遅延，モデル化誤差などの無い理想的な状況では1のときに臨界減衰となり，"\
            + "オーバーシュートなく最速で目標位置に収束します．"
        self.hor_damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "Horizontal damping ratio (Translation controller)",
            hor_damp_ratio_description,
            decimals=2,
            minimum=math.sqrt(0.5),
            default=1.,
        )
        self._rows.addWidget(self.hor_damp_ratio)

        ver_natural_freq_description = "PD制御における垂直方向の自然周波数．"\
            + "大きいほど応答速度が速くなりますが，"\
            + "大きすぎると遅延やモデル化誤差などの要因により振動が発生する恐れがあります．"
        self.ver_natural_freq = ParamGetterWidget_DoubleSpinBox(
            "Vertical natural frequency (Translation controller)",
            ver_natural_freq_description,
            decimals=2,
            minimum=0.1,
            default=2.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.ver_natural_freq)

        ver_damp_ratio_description = "PD制御における垂直方向の減衰比．"\
            + "大きいほどオーバーシュートが小さくなりますが，"\
            + "大きすぎると速度変化に過剰に反応して振動が発生する恐れがあります．"\
            + "逆に小さいほど応答速度が速くなりますが，"\
            + "小さすぎるとオーバーシュートが大きくなり目標位置付近での振動が大きくなります．"\
            + "遅延，モデル化誤差などの無い理想的な状況では1のときに臨界減衰となり，"\
            + "オーバーシュートなく最速で目標位置に収束します．"
        self.ver_damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "Vertical damping ratio (Translation controller)",
            ver_damp_ratio_description,
            decimals=2,
            minimum=math.sqrt(0.5),
            default=1.,
        )
        self._rows.addWidget(self.ver_damp_ratio)

        max_hor_vel_description = "水平方向の最大速度．"
        self.max_hor_vel = ParamGetterWidget_DoubleSpinBox(
            "Maximum horizontal velocity (Translation controller)",
            max_hor_vel_description,
            decimals=1,
            minimum=1.,
            maximum=10.,
            default=3.,
            suffix=" m/s",
        )
        self._rows.addWidget(self.max_hor_vel)

        max_ver_vel_description = "垂直方向の最大速度．"
        self.max_ver_vel = ParamGetterWidget_DoubleSpinBox(
            "Maximum vertical velocity (Translation controller)",
            max_ver_vel_description,
            decimals=1,
            minimum=1.,
            maximum=10.,
            default=3.,
            suffix=" m/s",
        )
        self._rows.addWidget(self.max_ver_vel)

        max_hor_acc_description = "水平方向の最大加速度．"
        self.max_hor_acc = ParamGetterWidget_DoubleSpinBox(
            "Maximum horizontal acceleration (Translation controller)",
            max_hor_acc_description,
            decimals=1,
            minimum=1.,
            maximum=10.,
            default=5.,
            suffix=" m/s^2",
        )
        self._rows.addWidget(self.max_hor_acc)

        max_ver_acc_description = "垂直方向の最大加速度．"
        self.max_ver_acc = ParamGetterWidget_DoubleSpinBox(
            "Maximum vertical acceleration (Translation controller)",
            max_ver_acc_description,
            decimals=1,
            minimum=1.,
            maximum=10.,
            default=5.,
            suffix=" m/s^2",
        )
        self._rows.addWidget(self.max_ver_acc)

        pred_horizon_description = "モデル予測制御の予測区間の長さ．"\
            + "理論的には大きいほど制御性能が良くなりますが，"\
            + "大きくするほど離散化誤差や計算機の数値誤差が大きくなります．"\
            + "最低でもシステム (ここでは姿勢制御) の応答時間の数倍以上にすべきだと言われています．"
        self.pred_horizon = ParamGetterWidget_DoubleSpinBox(
            "Predictin horizon (Rotation controller)",
            pred_horizon_description,
            decimals=2,
            minimum=0.1,
            maximum=3.,
            default=1.,
            suffix=" s",
        )
        self._rows.addWidget(self.pred_horizon)

        pred_steps_description = "モデル予測制御の予測区間の分割数．"\
            + "大きいほど離散化誤差が小さくなりますが，計算量は分割数の3乗に比例します．"
        self.pred_steps = ParamGetterWidget_SpinBox(
            "Prediction steps (Rotation controller)",
            pred_steps_description,
            minimum=1,
            maximum=30,
            default=10,
        )
        self._rows.addWidget(self.pred_steps)

        attitude_decay_description = "モデル予測制御における，ロール角とピッチ角の参照値の追従時定数．"\
            + "大きいほど目標値に滑らかに追従します．"
        self.attitude_decay = ParamGetterWidget_DoubleSpinBox(
            "Attitude decay time constant (Rotation controller)",
            attitude_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.2,
            suffix=" s",
        )
        self._rows.addWidget(self.attitude_decay)

        heading_decay_description = "モデル予測制御における，ヨー角の参照値の追従時定数．"\
            + "大きいほど目標値に滑らかに追従します．"
        self.heading_decay = ParamGetterWidget_DoubleSpinBox(
            "Heading decay time constant (Rotation controller)",
            heading_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.2,
            suffix=" s",
        )
        self._rows.addWidget(self.heading_decay)

        angvel_decay_description = "モデル予測制御における，角速度の参照値の追従時定数．"\
            + "大きいほど目標値に滑らかに追従します．"
        self.angvel_decay = ParamGetterWidget_DoubleSpinBox(
            "Angular velocity decay time constant (Rotation controller)",
            angvel_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.,
            suffix=" s",
        )
        self._rows.addWidget(self.angvel_decay)

        attitude_weight_description = "モデル予測制御における，ロール角とピッチ角の重み．"
        self.attitude_weight = ParamGetterWidget_SpinBox(
            "Attitude weight (Rotation controller)",
            attitude_weight_description,
            minimum=1,
            maximum=100,
            default=100,
        )
        self._rows.addWidget(self.attitude_weight)

        heading_weight_description = "モデル予測制御における，ヨー角の重み．"
        self.heading_weight = ParamGetterWidget_SpinBox(
            "Heading weight (Rotation controller)",
            heading_weight_description,
            minimum=1,
            maximum=100,
            default=10,
        )
        self._rows.addWidget(self.heading_weight)

        angvel_weight_description = "モデル予測制御における，角速度の重み．"
        self.angvel_weight = ParamGetterWidget_SpinBox(
            "Angular velocity weight (Rotation controller)",
            angvel_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.angvel_weight)

        thrust_weight_exp_description = "モデル予測制御における，プロペラ推力の重みの常用対数．"
        self.thrust_weight_exp = ParamGetterWidget_SpinBox(
            "Thrust weight level (Rotation controller)",
            thrust_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_weight_exp)

        thrust_rate_weight_exp_description = "モデル予測制御における，プロペラ推力の変化率の重みの常用対数．"
        self.thrust_rate_weight_exp = ParamGetterWidget_SpinBox(
            "Thrust rate weight level (Rotation controller)",
            thrust_rate_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_rate_weight_exp)

    def is_applicable(self) -> bool:
        return True  # TODO

    def is_valid(self) -> bool:
        if self.attitude_decay.get() > self.pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of attitude is greater the prediction horizon.",
            )
            return False
        if self.attitude_decay.get() > self.pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of heading is greater the prediction horizon.",
            )
            return False
        if self.attitude_decay.get() > self.pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of angular velocity is greater the prediction horizon.",
            )
            return False

        return True

    def parameter_dict(self) -> dict:
        res = dict()
        res["tobas_multirotor_controller"] = {
            "horizontal_natural_frequency": self.hor_natural_freq.get(),
            "horizontal_damping_ratio": self.hor_damp_ratio.get(),
            "vertical_natural_frequency": self.ver_natural_freq.get(),
            "vertical_damping_ratio": self.ver_damp_ratio.get(),
            "max_horizontal_velocity": self.max_hor_vel.get(),
            "max_vertical_velocity": self.max_ver_vel.get(),
            "max_horizontal_accel": self.max_hor_acc.get(),
            "max_vertical_accel": self.max_ver_acc.get(),
            "prediction_horizon": self.pred_horizon.get(),
            "prediction_steps": self.pred_steps.get(),
            "attitude_decay": self.attitude_decay.get(),
            "heading_decay": self.heading_decay.get(),
            "angular_velocity_decay": self.angvel_decay.get(),
            "attitude_weight": self.attitude_weight.get(),
            "heading_weight": self.heading_weight.get(),
            "angular_velocity_weight": self.angvel_weight.get(),
            "thrust_weight_exp": self.thrust_weight_exp.get(),
            "thrust_rate_weight_exp": self.thrust_rate_weight_exp.get(),
        }

        return res
