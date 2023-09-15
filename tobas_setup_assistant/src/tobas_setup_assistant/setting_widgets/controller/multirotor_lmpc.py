from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.messages import q_error_named
from kdl_sympy.frames import Vector

from tobas_msgs.msg import (
    PositionYaw,
    VelocityYaw,
    AccelerationYaw,
    RollPitchYawThrust,
    RollPitchYawrateThrust,
)

from ...parameter_getters import *
from ...common import *
from .base import BaseController


class MultirotorLMPC(BaseController):
    NAME = "Linear Model Predictive Control"

    CONTROLLER_PKG = "tobas_multirotor_controller"
    TAKEOFF_PKG = "tobas_multirotor_takeoff"
    LANDING_PKG = "tobas_multirotor_landing"

    COMMAND_MSGS = [
        PositionYaw.__name__,
        VelocityYaw.__name__,
        AccelerationYaw.__name__,
        RollPitchYawThrust.__name__,
        RollPitchYawrateThrust.__name__,
    ]

    # Dynamic Parameters
    HOR_NATURAL_FREQ = "horizontal_natural_frequency"
    HOR_DAMP_RATIO = "horizontal_damping_ratio"
    VER_NATURAL_FREQ = "vertical_natural_frequency"
    VER_DAMP_RATIO = "vertical_damping_ratio"
    MAX_HOR_VEL = "max_horizontal_velocity"
    MAX_VER_VEL = "max_vertical_velocity"
    MAX_HOR_ACC = "max_horizontal_accel"
    MAX_VER_ACC = "max_vertical_accel"
    PRED_HORIZON = "prediction_horizon"
    PRED_STEPS = "prediction_steps"
    ATTITUDE_DECAY = "attitude_decay"
    HEADING_DECAY = "heading_decay"
    ANGVEL_DECAY = "angular_velocity_decay"
    ATTITUDE_WEIGHT = "attitude_weight"
    HEADING_WEIGHT = "heading_weight"
    ANGVEL_WEIGHT = "angular_velocity_weight"
    THRUST_RATE_WEIGHT_LOG10 = "thrust_rate_weight_log10"

    MIN_NUM_PROP = 3

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "位置制御にPD制御，姿勢制御に線形モデル予測制御を用いた制御器です．"
        super().__init__(main, abst_text)

        config = self._get_param_config(self.HOR_NATURAL_FREQ)
        self._hor_natural_freq = ParamGetterWidget_DoubleSpinBox(
            "Horizontal natural frequency (Translation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" Hz",
        )
        self._rows.addWidget(self._hor_natural_freq)

        config = self._get_param_config(self.HOR_DAMP_RATIO)
        self._hor_damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "Horizontal damping ratio (Translation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._hor_damp_ratio)

        config = self._get_param_config(self.VER_NATURAL_FREQ)
        self._ver_natural_freq = ParamGetterWidget_DoubleSpinBox(
            "Vertical natural frequency (Translation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" Hz",
        )
        self._rows.addWidget(self._ver_natural_freq)

        config = self._get_param_config(self.VER_DAMP_RATIO)
        self._ver_damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "Vertical damping ratio (Translation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._ver_damp_ratio)

        config = self._get_param_config(self.MAX_HOR_VEL)
        self._max_hor_vel = ParamGetterWidget_DoubleSpinBox(
            "Maximum horizontal velocity (Translation controller)",
            config["description"],
            decimals=1,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" m/s",
        )
        self._rows.addWidget(self._max_hor_vel)

        config = self._get_param_config(self.MAX_VER_VEL)
        self._max_ver_vel = ParamGetterWidget_DoubleSpinBox(
            "Maximum vertical velocity (Translation controller)",
            config["description"],
            decimals=1,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" m/s",
        )
        self._rows.addWidget(self._max_ver_vel)

        config = self._get_param_config(self.MAX_HOR_ACC)
        self._max_hor_acc = ParamGetterWidget_DoubleSpinBox(
            "Maximum horizontal acceleration (Translation controller)",
            config["description"],
            decimals=1,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" m/s^2",
        )
        self._rows.addWidget(self._max_hor_acc)

        config = self._get_param_config(self.MAX_VER_ACC)
        self._max_ver_acc = ParamGetterWidget_DoubleSpinBox(
            "Maximum vertical acceleration (Translation controller)",
            config["description"],
            decimals=1,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" m/s^2",
        )
        self._rows.addWidget(self._max_ver_acc)

        config = self._get_param_config(self.PRED_HORIZON)
        self._pred_horizon = ParamGetterWidget_DoubleSpinBox(
            "Predictin horizon (Rotation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" s",
        )
        self._rows.addWidget(self._pred_horizon)

        config = self._get_param_config(self.PRED_STEPS)
        self._pred_steps = ParamGetterWidget_SpinBox(
            "Prediction steps (Rotation controller)",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._pred_steps)

        config = self._get_param_config(self.ATTITUDE_DECAY)
        self._attitude_decay = ParamGetterWidget_DoubleSpinBox(
            "Attitude decay time constant (Rotation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" s",
        )
        self._rows.addWidget(self._attitude_decay)

        config = self._get_param_config(self.HEADING_DECAY)
        self._heading_decay = ParamGetterWidget_DoubleSpinBox(
            "Heading decay time constant (Rotation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" s",
        )
        self._rows.addWidget(self._heading_decay)

        config = self._get_param_config(self.ANGVEL_DECAY)
        self._angvel_decay = ParamGetterWidget_DoubleSpinBox(
            "Angular velocity decay time constant (Rotation controller)",
            config["description"],
            decimals=2,
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
            suffix=" s",
        )
        self._rows.addWidget(self._angvel_decay)

        config = self._get_param_config(self.ATTITUDE_WEIGHT)
        self._attitude_weight = ParamGetterWidget_SpinBox(
            "Attitude weight (Rotation controller)",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._attitude_weight)

        config = self._get_param_config(self.HEADING_WEIGHT)
        self._heading_weight = ParamGetterWidget_SpinBox(
            "Heading weight (Rotation controller)",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._heading_weight)

        config = self._get_param_config(self.ANGVEL_WEIGHT)
        self._angvel_weight = ParamGetterWidget_SpinBox(
            "Angular velocity weight (Rotation controller)",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._angvel_weight)

        config = self._get_param_config(self.THRUST_RATE_WEIGHT_LOG10)
        self._thrust_rate_weight_log10 = ParamGetterWidget_SpinBox(
            "Thrust rate weight level (Rotation controller)",
            config["description"],
            minimum=config["min"],
            maximum=config["max"],
            default=config["default"],
        )
        self._rows.addWidget(self._thrust_rate_weight_log10)

    @overrides
    def is_applicable(self) -> bool:
        # 固定翼は持たない
        fixed_wing = self._main.settings.fixed_wing
        if fixed_wing.has_fixed_wing.isChecked():
            return False

        # プロペラの個数条件
        prop_jnt_names = self._main.settings.propulsion_system.selected.joint_names()
        if len(prop_jnt_names) < self.MIN_NUM_PROP:
            return False

        # Z軸正方向のプロペラのみ
        # FIXME: 複数回の回転を含む場合，数値誤差により理論的には存在しないXY要素が発生するかもしれない
        # FIXME: 特殊なドローンの場合はZ成分にシンボルが含まれる可能性がある
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitZ()):
                return False

        return True

    @overrides
    def is_valid(self) -> bool:
        # 両方の回転方向のプロペラをもつ
        directions = set(self._main.settings.propulsion_system.selected.directions())
        assert len(directions) <= 2
        if len(directions) == 1:
            q_error_named(
                self._main,
                self.NAME,
                "All rotors have the same rotation direction. "
                "Rotors that rotate in both clockwise (CW) and counterclockwise (CCW) are required.",
            )
            return False

        if self._attitude_decay.get() > self._pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of attitude is greater the prediction horizon.",
            )
            return False
        if self._attitude_decay.get() > self._pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of heading is greater the prediction horizon.",
            )
            return False
        if self._attitude_decay.get() > self._pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of angular velocity is greater the prediction horizon.",
            )
            return False

        return True

    @overrides
    def parameter_dict(self) -> dict:
        res = dict()
        res["tobas_multirotor_controller"] = {
            self.HOR_NATURAL_FREQ: self._hor_natural_freq.get(),
            self.HOR_DAMP_RATIO: self._hor_damp_ratio.get(),
            self.VER_NATURAL_FREQ: self._ver_natural_freq.get(),
            self.VER_DAMP_RATIO: self._ver_damp_ratio.get(),
            self.MAX_HOR_VEL: self._max_hor_vel.get(),
            self.MAX_VER_VEL: self._max_ver_vel.get(),
            self.MAX_HOR_ACC: self._max_hor_acc.get(),
            self.MAX_VER_ACC: self._max_ver_acc.get(),
            self.PRED_HORIZON: self._pred_horizon.get(),
            self.PRED_STEPS: self._pred_steps.get(),
            self.ATTITUDE_DECAY: self._attitude_decay.get(),
            self.HEADING_DECAY: self._heading_decay.get(),
            self.ANGVEL_DECAY: self._angvel_decay.get(),
            self.ATTITUDE_WEIGHT: self._attitude_weight.get(),
            self.HEADING_WEIGHT: self._heading_weight.get(),
            self.ANGVEL_WEIGHT: self._angvel_weight.get(),
            self.THRUST_RATE_WEIGHT_LOG10: self._thrust_rate_weight_log10.get(),
        }

        return res
