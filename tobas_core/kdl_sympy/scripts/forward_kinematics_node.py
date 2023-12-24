#!/usr/bin/env python3

import rospy
import os.path as osp
import sympy

from kdl_sympy.tree import Tree

if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

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
