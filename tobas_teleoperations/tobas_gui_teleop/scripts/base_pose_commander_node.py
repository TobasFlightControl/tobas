#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from tobas_gui_teleop.base_pose_commander import BasePoseCommander


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

    app = QApplication(sys.argv)

    main_widget = BasePoseCommander()
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
