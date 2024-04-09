#!/usr/bin/env python3

from tobas_rospy.utils import init_node
from tobas_rospy.plotjuggler_launcher import PlotJugglerLauncher


if __name__ == "__main__":
    init_node()
    node = PlotJugglerLauncher()
    node.run()
