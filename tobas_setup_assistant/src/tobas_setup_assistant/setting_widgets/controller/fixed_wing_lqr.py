from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from kdl_sympy.frames import Vector

from tobas_msgs.msg import SpeedRollDeltaPitch

from ...parameter_getters import *
from ...common import *
from .base import BaseController


class FixedWingLQR(BaseController):

    NAME = "Linear Quadratic Legulator"

    CONTROLLER_PKG = "tobas_fixed_wing_lqd"
    TAKEOFF_PKG = "TODO"  # TODO
    LANDING_PKG = "TODO"  # TODO

    COMMAND_MSGS = [SpeedRollDeltaPitch]

    MIN_PROP_NUM = 1

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "TODO"
        super().__init__(main, abst_text)

        forward_speed_weight_description = "最適制御における推進速度の重み．"
        self.forward_speed_weight = ParamGetterWidget_SpinBox(
            "Weight on forward speed",
            forward_speed_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.forward_speed_weight)

        alpha_weight_description = "最適制御における迎角の重み．"
        self.alpha_weight = ParamGetterWidget_SpinBox(
            "Weight on angle of attack",
            alpha_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.alpha_weight)

        beta_weight_description = "最適制御における推進速度の重み．"
        self.beta_weight = ParamGetterWidget_SpinBox(
            "Weight on angle of sideslip",
            beta_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.beta_weight)

        attitude_weight_description = "最適制御における姿勢角の重み．"
        self.attitude_weight = ParamGetterWidget_SpinBox(
            "Weight on attitude",
            attitude_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.attitude_weight)

        angvel_weight_description = "最適制御における角速度の重み．"
        self.angvel_weight = ParamGetterWidget_SpinBox(
            "Weight on angular velocity",
            angvel_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.angvel_weight)

        thrust_weight_exp_description = "最適制御における，プロペラ推力の重みの常用対数．"
        self.thrust_weight_exp = ParamGetterWidget_SpinBox(
            "Weight on thrust level",
            thrust_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_weight_exp)

        thrust_rate_weight_exp_description = "最適制御における，プロペラ推力の変化率の重みの常用対数．"
        self.thrust_rate_weight_exp = ParamGetterWidget_SpinBox(
            "Weight on thrust rate level",
            thrust_rate_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-1,
        )
        self._rows.addWidget(self.thrust_rate_weight_exp)

        deflection_weight_exp_description = "最適制御における，操舵角の重みの常用対数．"
        self.deflection_weight_exp = ParamGetterWidget_SpinBox(
            "Weight on deflection level",
            deflection_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.deflection_weight_exp)

        deflection_rate_weight_exp_description = "最適制御における，操舵角の変化率の重みの常用対数．"
        self.deflection_rate_weight_exp = ParamGetterWidget_SpinBox(
            "Weight on deflection rate level",
            deflection_rate_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-1,
        )
        self._rows.addWidget(self.deflection_rate_weight_exp)

    @overrides
    def is_applicable(self) -> bool:
        # 固定翼を持つ
        fixed_wing = self._main.settings.fixed_wing
        if not fixed_wing.has_fixed_wing.isChecked():
            return False

        # プロペラの個数条件
        prop_jnt_names = self._main.settings.propulsion_system.selected.joint_names()
        if len(prop_jnt_names) < self.MIN_PROP_NUM:
            return False

        # X軸正方向のプロペラのみ
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitX()):
                return False

        return True

    @overrides
    def is_valid(self) -> bool:
        # TODO: 制御面の数や符号などに関する条件
        return True

    @overrides
    def parameter_dict(self) -> dict:
        res = dict()
        res["tobas_fixed_wing_lqd"] = {
            "forward_speed_weight": self.forward_speed_weight.get(),
            "alpha_weight": self.alpha_weight.get(),
            "beta_weight": self.beta_weight.get(),
            "attitude_weight": self.attitude_weight.get(),
            "angular_velocity_weight": self.angvel_weight.get(),
            "thrust_weight_exp": self.thrust_weight_exp.get(),
            "thrust_rate_weight_exp": self.thrust_rate_weight_exp.get(),
            "deflection_weight_exp": self.deflection_weight_exp.get(),
            "deflection_rate_weight_exp": self.deflection_rate_weight_exp.get(),
        }

        return res
