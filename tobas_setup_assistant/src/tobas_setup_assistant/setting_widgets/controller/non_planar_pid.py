from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import override

from tobas_rqt_tools.messages import QMessageLevel, yes_or_no
from tobas_kdl_sympy.frames import Vector
from tobas_msgs.msg import PoseTwistAccelCommand

from ...common import PROP_TILT_TOL
from .base import BaseController


class NonPlanarPID(BaseController):
    NAME = "Non-Planar Multirotor PID"
    CONTROLLER_PKG = "tobas_np_pid"
    TAKEOFF_PKG = "tobas_dummy_pkg"  # TODO
    LANDING_PKG = "tobas_dummy_pkg"  # TODO
    MOVE_PKG = "tobas_dummy_pkg"  # TODO
    STABLIZE_MODE = PoseTwistAccelCommand.__name__
    ACROBAT_MODE = PoseTwistAccelCommand.__name__  # TODO
    ABST_TEXT = "This is a PID controller for non-planar multirotors."

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

        # 少なくとも1つのプロペラが鉛直上方向以外を向いている
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
                break
        else:
            return False

        return True

    @override
    def is_valid(self) -> bool:
        # 両方の回転方向のプロペラをもつ
        directions = set(self._main.propulsion_system.selected.directions())
        assert len(directions) <= 2
        if len(directions) == 1:
            if not yes_or_no(
                self._main,
                "All rotors have the same rotation direction. Is that OK?",
                QMessageLevel.WARN,
            ):
                return False

        # TODO

        return True

    @override
    def static_parameters(self) -> dict:
        return dict()
