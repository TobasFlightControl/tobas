#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from movedrone_setup_assistant import SetupAssistant

if __name__ == '__main__':
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

    app = QApplication(sys.argv)

    setup_assistant = SetupAssistant()
    setup_assistant.show()

    # Ctrl+Cを検出して即座に終了するための設定
    # 何故かこの位置に書いたときのみ機能する
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
