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
    VelocityYaw,
    AccelerationYaw,
    RollPitchYawThrust,
)

from ...parameter_getters import *
from ...common import *
from .base import BaseController


class MultirotorMpc(BaseController):
    NAME = "Linear Model Predictive Control"

    CONTROLLER_PKG = "tobas_mr_mpc"
    TAKEOFF_PKG = "tobas_multirotor_takeoff"
    LANDING_PKG = "tobas_multirotor_landing"

    COMMAND_MSGS = [
        VelocityYaw.__name__,
        AccelerationYaw.__name__,
        RollPitchYawThrust.__name__,
    ]

    MIN_NUM_PROP = 3

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "位置制御にLQR，姿勢制御に線形モデル予測制御を用いた制御器です．"
        super().__init__(main, abst_text)

        # TODO: 設定項目

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

        # TODO

        return True

    @overrides
    def parameter_dict(self) -> dict:
        return dict()  # TODO
