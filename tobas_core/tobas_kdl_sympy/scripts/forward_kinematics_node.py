#!/usr/bin/env python3

import rospy
import os.path as osp
import sympy

from tobas_kdl_sympy.tree import Tree
from tobas_rospy.utils import init_node

if __name__ == "__main__":
    init_node()

    tree = Tree()
    tree.load_from_param()

    link_name = rospy.get_param("~link_name")

    frame = tree.global_pose(link_name)
    x = frame.p.x
    y = frame.p.y
    z = frame.p.z
    roll, pitch, yaw = frame.M.get_rpy()
    print(f"X    : {sympy.simplify(x, chop=True)}")  # TODO: chopが機能しない
    print(f"Y    : {sympy.simplify(y, chop=True)}")
    print(f"Z    : {sympy.simplify(z, chop=True)}")
    print(f"Roll : {sympy.simplify(roll, chop=True)}")
    print(f"Pitch: {sympy.simplify(pitch, chop=True)}")
    print(f"Yaw  : {sympy.simplify(yaw, chop=True)}")
