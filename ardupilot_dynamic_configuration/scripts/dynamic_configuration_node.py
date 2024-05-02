#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospkg
from PyQt5.QtWidgets import QApplication

from tobas_rospy.utils import init_node
from tobas_rqt_tools.widgets import MainWidget
from tobas_rqt_tools.utils import handle_exception
from tobas_tools_py.constants import CONFIG_PATH

from ardupilot_dynamic_configuration.dynamic_configuration import DynamicConfigurationWidget
from ardupilot_dynamic_configuration.common import *


if __name__ == "__main__":
    init_node()
    app = QApplication(sys.argv)

    main_widget = MainWidget(
        CONFIG_PATH,
        PKG_NAME,
        TITLE,
        osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png"),
        DynamicConfigurationWidget(),
    )
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.excepthook = handle_exception

    sys.exit(app.exec())
