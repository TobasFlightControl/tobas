#!/usr/bin/env python3

import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from tobas_motor_test.pwm_publisher import PwmPublisherWidget


if __name__ == "__main__":
    rospy.init_node("pwm_publisher")

    app = QApplication(sys.argv)

    pwm_publisher = PwmPublisherWidget()
    pwm_publisher.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
