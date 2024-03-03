#!/usr/bin/env python3

import rospy
import rospkg
from dynamic_reconfigure import server

from state_estimation_eskf.cfg import StateEstimationEskfConfig


def dynamicReconfigureCb(
    config: StateEstimationEskfConfig, level: int
) -> StateEstimationEskfConfig:
    return config


if __name__ == "__main__":
    node_name = "state_estimation_eskf_param_server"
    rospy.init_node(node_name)

    srv = server.Server(StateEstimationEskfConfig, dynamicReconfigureCb)
    rospy.spin()
