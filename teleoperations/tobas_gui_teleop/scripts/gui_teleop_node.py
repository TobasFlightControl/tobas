#!/usr/bin/env python3

import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from tobas_gui_teleop.gui_teleop import GuiTeleopWidget


if __name__ == "__main__":
    rospy.init_node("gui_teleop")

    app = QApplication(sys.argv)

    gui_teleop = GuiTeleopWidget()
    gui_teleop.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
