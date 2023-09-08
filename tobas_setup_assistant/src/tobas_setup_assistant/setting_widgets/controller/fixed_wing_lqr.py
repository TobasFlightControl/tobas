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

    # Dynamic Parameters
    FORWARD_SPEED_WEIGHT = "forward_speed_weight"
    ALPHA_WEIGHT = "alpha_weight"
    BETA_WEIGHT = "beta_weight"
    ATTITUDE_WEIGHT = "attitude_weight"
    ANGVEL_WEIGHT = "angular_velocity_weight"
    THRUST_WEIGHT_LOG10 = "thrust_weight_log10"
    THRUST_RATE_WEIGHT_LOG10 = "thrust_rate_weight_log10"
    DEFLECTION_WEIGHT_LOG10 = "deflection_weight_log10"
    DEFLECTION_RATE_WEIGHT_LOG10 = "deflection_rate_weight_log10"

    MIN_NUM_PROP = 1
    MIN_NUM_CS = 2

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "TODO"
        super().__init__(main, abst_text)

        config = self._get_param_config(self.FORWARD_SPEED_WEIGHT)
        self._forward_speed_weight = ParamGetterWidget_SpinBox(
            "Weight on forward speed",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._forward_speed_weight)

        config = self._get_param_config(self.ALPHA_WEIGHT)
        self._alpha_weight = ParamGetterWidget_SpinBox(
            "Weight on angle of attack",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._alpha_weight)

        config = self._get_param_config(self.BETA_WEIGHT)
        self._beta_weight = ParamGetterWidget_SpinBox(
            "Weight on angle of sideslip",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._beta_weight)

        config = self._get_param_config(self.ATTITUDE_WEIGHT)
        self._attitude_weight = ParamGetterWidget_SpinBox(
            "Weight on attitude",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._attitude_weight)

        config = self._get_param_config(self.ANGVEL_WEIGHT)
        self._angvel_weight = ParamGetterWidget_SpinBox(
            "Weight on angular velocity",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._angvel_weight)

        config = self._get_param_config(self.THRUST_WEIGHT_LOG10)
        self._thrust_weight_log10 = ParamGetterWidget_SpinBox(
            "Weight on thrust level",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._thrust_weight_log10)

        config = self._get_param_config(self.THRUST_RATE_WEIGHT_LOG10)
        self._thrust_rate_weight_log10 = ParamGetterWidget_SpinBox(
            "Weight on thrust rate level",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._thrust_rate_weight_log10)

        config = self._get_param_config(self.DEFLECTION_WEIGHT_LOG10)
        self._deflection_weight_log10 = ParamGetterWidget_SpinBox(
            "Weight on deflection level",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._deflection_weight_log10)

        config = self._get_param_config(self.DEFLECTION_RATE_WEIGHT_LOG10)
        self._deflection_rate_weight_log10 = ParamGetterWidget_SpinBox(
            "Weight on deflection rate level",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._deflection_rate_weight_log10)

    @overrides
    def is_applicable(self) -> bool:
        # 固定翼を持つ
        fixed_wing = self._main.settings.fixed_wing
        if not fixed_wing.has_fixed_wing.isChecked():
            return False

        # 制御面の個数条件
        if fixed_wing.num_control_surfaces() < self.MIN_NUM_CS:
            return False

        # プロペラの個数条件
        prop_jnt_names = self._main.settings.propulsion_system.selected.joint_names()
        if len(prop_jnt_names) < self.MIN_NUM_PROP:
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
            self.FORWARD_SPEED_WEIGHT: self._forward_speed_weight.get(),
            self.ALPHA_WEIGHT: self._alpha_weight.get(),
            self.BETA_WEIGHT: self._beta_weight.get(),
            self.ATTITUDE_WEIGHT: self._attitude_weight.get(),
            self.ANGVEL_WEIGHT: self._angvel_weight.get(),
            self.THRUST_WEIGHT_LOG10: self._thrust_weight_log10.get(),
            self.THRUST_RATE_WEIGHT_LOG10: self._thrust_rate_weight_log10.get(),
            self.DEFLECTION_WEIGHT_LOG10: self._deflection_weight_log10.get(),
            self.DEFLECTION_RATE_WEIGHT_LOG10: self._deflection_rate_weight_log10.get(),
        }

        return res
