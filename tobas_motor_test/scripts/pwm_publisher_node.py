#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospkg
from PyQt5.QtWidgets import QApplication

from tobas_rospy.utils import init_node
from tobas_rqt_tools.widgets import MainWidget
from tobas_tools_py.constants import CONFIG_PATH

from tobas_motor_test.pwm_publisher import PwmPublisherWidget
from tobas_motor_test.common import *


if __name__ == "__main__":
    init_node()
    app = QApplication(sys.argv)

    main_widget = MainWidget(
        CONFIG_PATH,
        PKG_NAME,
        TITLE,
        osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png"),
        PwmPublisherWidget(),
    )
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec())
