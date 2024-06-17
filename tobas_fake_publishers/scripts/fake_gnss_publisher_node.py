#!/usr/bin/env python3

import rospy
from tobas_rospy.utils import init_node
from tobas_fake_publishers.fake_gnss_publisher import FakeGNSSPublisher

if __name__ == "__main__":
    init_node()
    node = FakeGNSSPublisher()
    rospy.spin()
