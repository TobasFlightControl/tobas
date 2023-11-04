from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.messages import *
from kdl_sympy.frames import Vector

from tobas_msgs.msg import PosVelAccYaw, RollPitchYawThrust

from ...parameter_getters import *
from ...common import *
from .base import BaseController


class NonPlanarPid(BaseController):
    NAME = "Non-Planar Multirotor PID"

    CONTROLLER_PKG = "tobas_np_pid"
    TAKEOFF_PKG = "tobas_dummy_pkg"  # TODO
    LANDING_PKG = "tobas_dummy_pkg"  # TODO
    PARAM_SERVER_NODE = "tobas_np_pid"

    COMMAND_MSGS = [PosVelAccYaw.__name__, RollPitchYawThrust.__name__]

    MIN_NUM_PROP = 3

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "非平面ロータ配置マルチコプターのためのPID制御器です．"
        super().__init__(main, abst_text)

        # TODO: 設定項目

    @overrides
    def define_connections(self) -> None:
        super().define_connections()

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

        # 少なくとも1つのプロペラが鉛直上方向以外を向いている
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
                break
        else:
            return False

        return True

    @overrides
    def is_valid(self) -> bool:
        # 両方の回転方向のプロペラをもつ
        directions = set(self._main.settings.propulsion_system.selected.directions())
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

    @overrides
    def parameter_dict(self) -> dict:
        return super().parameter_dict()  # TODO
