from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from typing import List, Tuple, Union
from urdf_parser_py.urdf import Link, Joint, Pose, Inertia, Inertial
from PyQt5.QtCore import QObject

from tobas_std_tools_py.sequence import is_unique
from tobas_rqt_tools.messages import q_error
from tobas_kdl_sympy.tree import Tree
from tobas_kdl_sympy.frames import Vector, Frame
from tobas_kdl_sympy.joint import JointType, HardwareInterface


class URDFParser(QObject):
    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._tree = Tree()

    def load_from_param(self) -> bool:
        try:
            self._tree.load_from_param()
        except Exception as e:
            q_error(self._main, f"Failed to load robot: {e}")
            return False

        if not self._is_valid_robot():
            return False

        return True

    def get_links(self) -> List[Link]:
        return self._tree.get_links()

    def get_joints(self) -> List[Joint]:
        return self._tree.get_joints()

    def get_root(self) -> Link:
        return self._tree.get_root()

    def get_link(self, link_name: str) -> Link:
        return self._tree.get_link(link_name)

    def get_joint(self, link_name: str) -> Union[Joint, None]:
        return self._tree.get_joint(link_name)

    def get_parent(self, link_name: str) -> Link:
        return self._tree.get_parent(link_name)

    def get_children(self, link_name: str) -> List[Tuple[str, str]]:
        return self._tree.get_children(link_name)

    def hardware_interface(self, jnt_name: str) -> Union[HardwareInterface, None]:
        return self._tree.hardware_interface(jnt_name)

    def is_end_link(self, link_name: str) -> bool:
        return self._tree.is_end_link(link_name)

    def is_fixed_link(self, link_name: str) -> bool:
        return self._tree.is_fixed_link(link_name)

    def link_exists(self, link_name: str) -> bool:
        return self._tree.link_exists(link_name)

    def joint_exists(self, jnt_name: str) -> bool:
        return self._tree.joint_exists(jnt_name)

    def link_names(self) -> List[str]:
        return self._tree.link_names()

    def joint_names(self) -> List[str]:
        return self._tree.joint_names()

    def mobile_joint_names(self) -> List[str]:
        """可動関節名のリストを返す．"""
        res = []
        for jnt_name in self._tree.joint_names():
            if not self._tree.is_fixed_joint(jnt_name):
                res.append(jnt_name)
        return res

    def search_joint_type(self, jnt_type: JointType) -> List[str]:
        """指定したタイプのジョイント名のリストを返す．"""
        res = []
        for joint in self._tree.get_joints():
            if joint.type == jnt_type:
                res.append(joint.name)
        return res

    def link_names_with_mobile_joint(self) -> List[str]:
        """可動関節をもつリンク名のリストを返す．"""
        mobile_joints = set(self.mobile_joint_names())
        res = []
        for link in self._tree.get_links():
            if link.name == self._tree.get_root().name:
                continue
            joint = self._tree.get_joint(link.name)
            if joint.name in mobile_joints:
                res.append(link.name)
        return res

    def link_names_available_in_gazebo(self) -> List[str]:
        """
        Gazeboで扱えるリンク名を返す．\\
        Gazeboの仕様で，ルートリンクまたは可動関節をもつリンク以外は省略されてしまう．
        """
        root_name = self.get_root().name
        return [root_name] + self.link_names_with_mobile_joint()

    def global_pose(self, link_name: str) -> Frame:
        return self._tree.global_pose(link_name)

    def global_axis(self, jnt_name: str) -> Vector:
        return self._tree.global_axis(jnt_name)

    def nwu_fixed_link_names(self) -> List[str]:
        """
        以下の条件を満たすリンクの名前の配列を返す．
        - ルートリンクに固定されている．
        - フレームの座標軸が XYZ = NWU に一致する．
        """
        root_link = self.get_root()
        return self._nwu_fixed_link_names_rec(root_link.name)

    def _nwu_fixed_link_names_rec(self, parent_name: str) -> List[str]:
        """parent以下の固定リンクの名前の配列を返す．"""
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
            if link.origin is not None and not link.origin.rpy != [0, 0, 0]:
                continue
            if joint.origin is not None and joint.origin.rpy != [0, 0, 0]:
                continue

            res += self._nwu_fixed_link_names_rec(child_name)

        return res

    def _is_valid_robot(self) -> bool:
        """有効なロボットかどうかを判定する．"""
        root_link = self.get_root()

        # リンク名とジョイント名が一意である
        if not is_unique(self.link_names()):
            q_error(self._main, f"Link names are not unique.")
            return False
        if not is_unique(self.joint_names()):
            q_error(self._main, f"Joint names are not unique.")
            return False

        # 多自由度関節を持たない
        for joint in self.get_joints():
            if joint.type in {JointType.PLANER, JointType.FLOATING}:
                q_error(self._main, f"Invalid joint type: {joint.type}")
                return False

        # ルートリンクのフレーム座標軸が XYZ = NWU に一致する
        origin: Pose = root_link.origin
        if origin is not None and any(angle != 0 for angle in origin.rpy):
            q_error(self._main, "The frame of the root link must coincide with the NWU coordinate axis.")
            return False

        # ルートリンクがInertialを持たない
        if root_link.inertial is not None:
            inertial: Inertial = root_link.inertial
            mass = inertial.mass
            inertia: Inertia = inertial.inertia

            if mass != 0 or any(row != [0, 0, 0] for row in inertia.to_matrix()):
                q_error(
                    self._main,
                    "The root link has an inertia specified in the URDF, "
                    + "but KDL does not support a root link with an inertia. "
                    + "As a workaround, you can add an extra dummy link to your URDF.",
                )
                return False

        return True
