from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import override

from tobas_rqt_tools.messages import q_error_named
from tobas_kdl_sympy.frames import Vector
from tobas_msgs.msg import PosVelAccYaw, RollPitchYawThrust

from ...common import PROP_TILT_TOL
from .base import BaseController


class MultirotorMpc(BaseController):
    NAME = "Multirotor MPC"
    CONTROLLER_PKG = "tobas_mr_mpc"
    TAKEOFF_PKG = "tobas_multirotor_takeoff"
    LANDING_PKG = "tobas_multirotor_landing"
    MOVE_PKG = "tobas_multirotor_move"
    STABLIZE_MODE = PosVelAccYaw.__name__
    ACROBAT_MODE = RollPitchYawThrust.__name__
    ABST_TEXT = (
        "This is a controller for planar multirotors, "
        "utilizing LQR for position control and linear model predictive control for attitude control."
    )

    MIN_NUM_PROP = 3

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def dump_settings(self) -> dict:
        return dict()

    @override
    def load_settings(self, data: dict) -> None:
        pass

    @override
    def is_applicable(self) -> bool:
        # 固定翼は持たない
        fixed_wing = self._main.fixed_wing
        if fixed_wing.has_fixed_wing.isChecked():
            return False

        # プロペラの個数条件
        prop_jnt_names = self._main.propulsion_system.selected.joint_names()
        if len(prop_jnt_names) < self.MIN_NUM_PROP:
            return False

        # Z軸正方向のプロペラのみ
        # FIXME: 特殊なドローンの場合はZ成分にシンボルが含まれる可能性がある
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
                return False

        return True

    @override
    def is_valid(self) -> bool:
        # 両方の回転方向のプロペラをもつ
        directions = set(self._main.propulsion_system.selected.directions())
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

    @override
    def static_parameters(self) -> dict:
        return dict()
