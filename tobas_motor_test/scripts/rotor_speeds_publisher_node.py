#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from tobas_motor_test.rotor_speeds_publisher import RotorSpeedsPublisherWidget


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

    app = QApplication(sys.argv)

    rotor_speeds_publisher = RotorSpeedsPublisherWidget()
    rotor_speeds_publisher.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
