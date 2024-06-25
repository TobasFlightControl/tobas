#!/usr/bin/env python3

import rospy
from tobas_msgs.msg import Message, Gps


class GnssStateChecker:
    def __init__(self) -> None:
        self._message_pub = rospy.Publisher("message", Message, queue_size=1)
        self._gps_sub = rospy.Subscriber("gps", Gps, self._gps_callback, queue_size=1)

    def _gps_callback(self, gps: Gps) -> None:
        message = Message()
        message.header.stamp = gps.header.stamp
        message.name = rospy.get_name()

        if gps.fix_type == Gps.FIX_3D:
            message.level = Message.INFO
            message.message = "GNSS Fix"
        else:
            message.level = Message.WARN
            message.message = "GNSS No Fix"

        self._message_pub.publish(message)


if __name__ == "__main__":
    rospy.init_node("gnss_state_checker")
    node = GnssStateChecker()
    rospy.spin()
