#!/usr/bin/env python3

import rospy
from dynamic_reconfigure import server

from state_estimation_cascade.cfg import StateEstimationCascadeConfig


def dynamicReconfigureCb(config: StateEstimationCascadeConfig, level: int) -> StateEstimationCascadeConfig:
    return config


if __name__ == "__main__":
    node_name = "state_estimation_cascade_param_server"
    rospy.init_node(node_name)

    srv = server.Server(StateEstimationCascadeConfig, dynamicReconfigureCb)
    rospy.spin()
