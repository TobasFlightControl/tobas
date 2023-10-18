#!/usr/bin/env python3

import rospy
import rospkg
from dynamic_reconfigure import server

from tobas_mr_arducopter.cfg import ControllerConfig


def dynamicReconfigureCb(config: ControllerConfig, level: int) -> ControllerConfig:
    return config


if __name__ == "__main__":
    node_name = rospkg.get_package_name(__file__) + "_param_server"
    rospy.init_node(node_name)

    srv = server.Server(ControllerConfig, dynamicReconfigureCb)
    rospy.spin()
