from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from tobas_kdl_sympy.frames import Vector
from tobas_msgs.msg import SpeedRollDeltaPitch

from ...common import PROP_TILT_TOL
from .base import BaseController


class FixedWingLQR(BaseController):
    NAME = "Fixed Wing LQR"
    CONTROLLER_PKG = "tobas_fixed_wing_lqd"
    TAKEOFF_PKG = "tobas_dummy_pkg"  # TODO
    LANDING_PKG = "tobas_dummy_pkg"  # TODO
    MOVE_PKG = "tobas_dummy_pkg"  # TODO
    STABLIZE_MODE = SpeedRollDeltaPitch.__name__
    ACROBAT_MODE = SpeedRollDeltaPitch.__name__  # TODO

    MIN_NUM_PROP = 1
    MIN_NUM_CS = 2

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = (
            "Control the fixed-wing aircraft using LQR (Linear Quadratic Regulator). "
            "While this method is computationally light, it does not consider hard constraints, "
            "which may lead to the issuance of commands outside the permissible range."
        )
        super().__init__(main, abst_text)

    @override
    def define_connections(self) -> None:
        pass

    @override
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
            if not axis.is_collinear(Vector.UnitX(), PROP_TILT_TOL):
                return False

        return True

    @override
    def is_valid(self) -> bool:
        # TODO: 制御面の数や符号などに関する条件
        return True
