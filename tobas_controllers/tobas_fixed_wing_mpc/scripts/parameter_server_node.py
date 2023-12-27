#!/usr/bin/env python3

import rospy
from dynamic_reconfigure import server

from tobas_fixed_wing_mpc.cfg import ControllerConfig


def dynamicReconfigureCb(config: ControllerConfig, level: int) -> ControllerConfig:
    return config


if __name__ == "__main__":
    node_name = "tobas_fixed_wing_mpc_param_server"
    rospy.init_node(node_name)

    srv = server.Server(ControllerConfig, dynamicReconfigureCb)
    rospy.spin()
