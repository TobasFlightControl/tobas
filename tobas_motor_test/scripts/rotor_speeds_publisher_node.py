#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospy
import rospkg
from PyQt5.QtWidgets import QApplication

from tobas_rqt_tools.widgets import MainWidget

from tobas_motor_test.rotor_speeds_publisher import RotorSpeedsPublisherWidget
from tobas_setup_assistant.common import *


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

    app = QApplication(sys.argv)

    main_widget = MainWidget(
        PKG_NAME,
        TITLE,
        osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png"),
        RotorSpeedsPublisherWidget(),
    )
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
