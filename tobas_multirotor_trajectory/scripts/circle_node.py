#!/usr/bin/env python3

import os.path as osp
import rospy

from tobas_multirotor_trajectory import FollowTrajectoryClient_Circle

if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)
    node = FollowTrajectoryClient_Circle()
    node.run()
