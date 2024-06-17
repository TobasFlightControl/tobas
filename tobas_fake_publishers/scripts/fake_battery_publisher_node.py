#!/usr/bin/env python3

import rospy
from tobas_rospy.utils import init_node
from tobas_fake_publishers.fake_battery_publisher import FakeBatteryPublisher

if __name__ == "__main__":
    init_node()
    node = FakeBatteryPublisher()
    rospy.spin()
