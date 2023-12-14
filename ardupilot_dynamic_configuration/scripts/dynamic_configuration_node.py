#!/usr/bin/env python3

import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from ardupilot_dynamic_configuration import DynamicConfigurationWidget


if __name__ == "__main__":
    rospy.init_node("ardupilot_dynamic_configuration")

    app = QApplication(sys.argv)

    main_widget = DynamicConfigurationWidget()
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
