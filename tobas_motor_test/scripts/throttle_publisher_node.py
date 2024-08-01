import os.path as osp
import sys
import signal
import rospkg
from PyQt5.QtWidgets import QApplication

from tobas_rclpy.utils import init_node
from tobas_rqt_tools.widgets import MainWidget
from tobas_rqt_tools.utils import handle_unexpected_exception

from tobas_motor_test.throttle_publisher import ThrottlePublisherWidget
from tobas_motor_test.common import TITLE, PKG_NAME


if __name__ == "__main__":
    init_node()
    app = QApplication(sys.argv)

    main_widget = MainWidget(
        TITLE,
        osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png"),
        ThrottlePublisherWidget(),
    )
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.excepthook = handle_unexpected_exception

    sys.exit(app.exec())
