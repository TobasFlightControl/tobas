#!/usr/bin/env python3

from tobas_rospy.utils import init_node
from tobas_rospy.dynamic_reconfigure import find_dynamic_reconfigure_servers


if __name__ == "__main__":
    init_node()

    servers = find_dynamic_reconfigure_servers()
    print(*servers, sep="\n")
