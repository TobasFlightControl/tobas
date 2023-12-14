#!/usr/bin/env python3

import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from tobas_motor_test.motor_test_gui import MotorTestGui


if __name__ == "__main__":
    rospy.init_node("motor_test_gui")

    app = QApplication(sys.argv)

    motor_test_gui = MotorTestGui()
    motor_test_gui.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
