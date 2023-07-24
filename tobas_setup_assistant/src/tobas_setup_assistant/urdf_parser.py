from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import rospy
from typing import List, Tuple
from urdf_parser_py.urdf import Link, Joint
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.messages import q_error
from kdl_sympy.frames import *
from kdl_sympy.tree import Tree
from kdl_sympy.joint import JointType


class URDFParser(QWidget):

    robot_model_updated = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._tree = Tree()

    def define_connections(self) -> None:
        self._main.settings.start.robot_model_loader.urdf_loaded.connect(self._on_urdf_loaded)

    @pyqtSlot()
    def _on_urdf_loaded(self) -> None:
        self._tree.load_from_param()
        if not self._is_valid_robot():
            return

        rospy.loginfo("Robot model is loaded successfully.")
        self.robot_model_updated.emit()

    def get_links(self) -> List[Link]:
        return self._tree.get_links()

    def get_joints(self) -> List[Joint]:
        return self._tree.get_joints()

    def get_root(self) -> Link:
        return self._tree.get_root()

    def get_link(self, link_name: str) -> Link:
        return self._tree.get_link(link_name)

    def get_joint(self, link_name: str) -> Joint:
        return self._tree.get_joint(link_name)

    def get_parent(self, link_name: str) -> Link:
        return self._tree.get_parent(link_name)

    def get_children(self, link_name: str) -> List[Tuple[str, str]]:
        return self._tree.get_children(link_name)

    def is_end_link(self, link_name: str) -> bool:
        return self._tree.is_end_link(link_name)

    def link_exists(self, link_name: str) -> bool:
        return self._tree.link_exists(link_name)

    def joint_exists(self, joint_name: str) -> bool:
        return self._tree.joint_exists(joint_name)

    def link_names(self) -> List[str]:
        return self._tree.link_names()

    def posture_defining_joint_names(self) -> List[str]:
        """
        ロボットの形状を決めるのに必要な関節名のリストを返す．\\
        プロペラに設定されていない可動リンクがあるかどうかを調べる．
        """
        rotary_wing_joints = set(self._main.settings.rotary_wings.selected.joint_names())
        res = []

        for joint in self.get_joints():
            if (not joint.name in rotary_wing_joints) and (not self._tree.is_fixed_joint(joint.name)):
                res.append(joint.name)

        return res

    def global_pose(self, link_name: str) -> Frame:
        return self._tree.global_pose(link_name)

    def global_axis(self, joint_name: str) -> Vector:
        return self._tree.global_axis(joint_name)

    def nwu_fixed_link_names(self) -> List[str]:
        """
        以下の条件を満たすリンクの名前の配列を返す．
        - ルートリンクに固定されている．
        - フレームの座標軸が XYZ = NWU に一致する．
        """
        root_link = self.get_root()
        return self._nwu_fixed_link_names_rec(root_link.name)

    def _nwu_fixed_link_names_rec(self, parent_name: str) -> List[str]:
        """ parent以下の固定リンクの名前の配列を返す． """
        res = [parent_name]

        if self.is_end_link(parent_name):
            return res

        for _, child_name in self.get_children(parent_name):
            link = self.get_link(child_name)
            joint = self.get_joint(child_name)

            # 固定関節であることを保証
            if joint.type != JointType.FIXED:
                continue

            # 親フレームと子フレームの回転が一致していることを保証
            if link.origin is not None and link.origin.rpy != [0, 0, 0]:
                continue
            if joint.origin is not None and joint.origin.rpy != [0, 0, 0]:
                continue

            res += self._nwu_fixed_link_names_rec(child_name)

        return res

    def _is_valid_robot(self) -> bool:
        """ 有効なロボットかどうかを判定する． """
        # 多自由度関節を持たないことを保証
        for joint in self.get_joints():
            if joint.type in {JointType.PLANER, JointType.FLOATING}:
                q_error(self._main, f'Invalid joint type: {joint.type}')
                return False

        # ルートリンクのフレーム座標軸が XYZ = NWU に一致することを保証
        root_link = self.get_root()
        if root_link.origin is not None and root_link.origin.rpy != [0, 0, 0]:
            q_error(
                self._main,
                "The frame of the root link must coincide with the NWU coordinate axis.",
            )
            return False

        return True
