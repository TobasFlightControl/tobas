import sympy
from sympy import Symbol
from typing import List, Tuple, Dict, Union
from urdf_parser_py.urdf import *

from .frames import *
from .joint import *


class Tree:
    def __init__(self) -> None:
        self._robot = Robot()
        self._q_map: Dict[str, Union[Symbol, int]] = {}

    def load_from_param(self, param: str = "robot_description") -> None:
        self._robot = Robot.from_parameter_server(param)

        for i, joint in enumerate(self.get_joints()):
            if joint.type == JointType.FIXED:
                self._q_map[joint.name] = 0
            elif joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
                self._q_map[joint.name] = sympy.symbols(rf"\theta_{i}")
            elif joint.type == JointType.PRISMATIC:
                self._q_map[joint.name] = sympy.symbols(rf"d_{i}")
            else:
                raise RuntimeError(f"Invalid joint type: {joint.type}")

    def get_num_joints(self) -> int:
        """固定関節を含む全関節の個数．"""
        return len(self._robot.joints)

    def get_num_links(self) -> int:
        """リンクの個数．"""
        return len(self._robot.links)

    def get_links(self) -> List[Link]:
        return self._robot.links

    def get_joints(self) -> List[Joint]:
        return self._robot.joints

    def get_root(self) -> Link:
        root_name = self._robot.get_root()
        return self._robot.link_map[root_name]

    def get_link(self, link_name: str) -> Link:
        return self._robot.link_map[link_name]

    def get_joint(self, link_name: str) -> Union[Joint, None]:
        if link_name == self._robot.get_root():
            return None

        jnt_name, _ = self._robot.parent_map[link_name]
        return self._robot.joint_map[jnt_name]

    def get_angle(self, link_name: str) -> Union[Symbol, int]:
        joint = self.get_joint(link_name)
        return self._q_map[joint.name]

    def get_parent(self, link_name: str) -> Link:
        _, parent_name = self._robot.parent_map[link_name]
        return self._robot.link_map[parent_name]

    def get_children(self, link_name: str) -> List[Tuple[str, str]]:
        return self._robot.child_map[link_name]

    def hardware_interface(self, jnt_name: str) -> Union[HardwareInterface, None]:
        for transmission in self._robot.transmissions:
            for joint in transmission.joints:
                if joint.name == jnt_name:
                    if len(joint.hardwareInterfaces) == 0:
                        raise RuntimeError("Hardware interface is not specified.")
                    elif len(joint.hardwareInterfaces) >= 2:
                        raise RuntimeError(
                            "Multiple hardware interfaces is not supported."
                        )
                    hi: str = joint.hardwareInterfaces[0]
                    if hi == HardwareInterface.POSITION.value:
                        return HardwareInterface.POSITION
                    elif hi == HardwareInterface.VELOCITY.value:
                        return HardwareInterface.VELOCITY
                    elif hi == HardwareInterface.EFFORT.value:
                        return HardwareInterface.EFFORT
                    else:
                        raise RuntimeError(f"Unknown hardware interface: {hi}")

        return None

    def is_end_link(self, link_name: str) -> bool:
        assert link_name in self._robot.link_map.keys()
        return link_name not in self._robot.child_map.keys()

    def is_fixed_link(self, link_name: str) -> bool:
        """リンクがルートに固定されている場合にTrueを返す．"""
        assert link_name in self._robot.link_map.keys()

        if link_name == self._robot.get_root():
            return True

        _, parent_name = self._robot.parent_map[link_name]
        return self.is_fixed_link(parent_name)

    def is_fixed_joint(self, jnt_name: str) -> bool:
        joint: Joint = self._robot.joint_map[jnt_name]
        return joint.type == JointType.FIXED

    def link_exists(self, link_name: str) -> bool:
        for link in self.get_links():
            if link.name == link_name:
                return True
        return False

    def joint_exists(self, jnt_name: str) -> bool:
        for joint in self.get_joints():
            if joint.name == jnt_name:
                return True
        return False

    def link_names(self) -> List[str]:
        """全てのリンクの名前を返す．"""
        links = self.get_links()
        return [link.name for link in links]

    def joint_names(self) -> List[str]:
        """全てのジョイントの名前を変えす．"""
        joints = self.get_joints()
        return [joint.name for joint in joints]

    def local_pose(self, link_name: str) -> Frame:
        """親リンクに対する位置姿勢を求める．"""
        if link_name == self._robot.get_root():
            return Frame.Identity()  # FIXME: Identityじゃないパターンがあるかも

        # リンクの原点はジョイントに一致していることを保証
        link = self.get_link(link_name)
        assert link.origin is None

        joint = self.get_joint(link_name)
        T_parent_joint = self._parent_to_joint(joint.name)
        T_joint_link = self._joint_to_link(joint.name)
        return T_parent_joint * T_joint_link

    def global_pose(self, link_name: str) -> Frame:
        """Forward Kinematics."""
        return self._recursive_fk(link_name)

    def local_axis(self, jnt_name: str) -> Union[Vector, None]:
        joint: Joint = self._robot.joint_map[jnt_name]
        return None if joint.axis is None else Vector(*joint.axis)

    def global_axis(self, jnt_name: str) -> Vector:
        joint: Joint = self._robot.joint_map[jnt_name]
        if joint.axis is None:
            return None

        # ルートからジョイント原点までのTFを求める．
        T_W_Parent = self._recursive_fk(joint.parent)
        T_Parent_Joint = self._parent_to_joint(jnt_name)
        T_W_Joint = T_W_Parent * T_Parent_Joint

        local_axis = Vector(*joint.axis)
        return T_W_Joint.M * local_axis  # TFではなく回転のみであることに注意

    def _recursive_fk(self, link_name: str) -> Frame:
        cur_frame = self.local_pose(link_name)

        if link_name == self.get_root().name:
            return cur_frame

        parent = self.get_parent(link_name)
        return self._recursive_fk(parent.name) * cur_frame

    def _parent_to_joint(self, jnt_name: str) -> Frame:
        """親フレーム -> ジョイント原点"""
        joint: Joint = self._robot.joint_map[jnt_name]
        if joint.origin is None:
            return Frame.Identity()
        else:
            p_origin = Vector(*joint.origin.xyz)
            M_origin = Rotation.RPY(*joint.origin.rpy)
            return Frame(p_origin, M_origin)

    def _joint_to_link(self, jnt_name: str) -> Frame:
        """ジョイント原点 -> リンク原点"""
        joint: Joint = self._robot.joint_map[jnt_name]
        if joint.type == JointType.FIXED:
            return Frame.Identity()
        elif joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
            axis = Vector(*joint.axis)
            angle = self._q_map[joint.name]
            return Frame.Rot(Rotation.Rodrigues(axis, angle))
        elif joint.type == JointType.PRISMATIC:
            axis = Vector(*joint.axis)
            angle = self._q_map[joint.name]
            return Frame.Trans(axis * angle)
        else:
            raise RuntimeError(f"Invalid joint type: {joint.type}")
