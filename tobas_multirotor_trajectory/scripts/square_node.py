#!/usr/bin/env python3

from tobas_rospy.utils import init_node
from tobas_multirotor_trajectory import FollowTrajectoryClient_Square

if __name__ == "__main__":
    init_node()
    node = FollowTrajectoryClient_Square()
    node.run()
