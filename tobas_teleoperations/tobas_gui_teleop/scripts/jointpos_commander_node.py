import sys
import signal
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path
from PyQt5.QtWidgets import QApplication

from tobas_rqt_tools.widgets import MainWidget
from tobas_rqt_tools.utils import handle_unexpected_exception

from tobas_gui_teleop.jointpos_commander import JointPositionsCommanderWidget
from tobas_gui_teleop.common import PKG_NAME


def main(args=None):
    rclpy.init(args=args)
    node = Node("joint_position_commander")

    app = QApplication(sys.argv)

    main_widget = MainWidget(
        "Joint Position Commander",
        str(get_package_share_path(PKG_NAME) / "images/icon.png"),
        JointPositionsCommanderWidget(node),
    )
    main_widget.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.excepthook = handle_unexpected_exception

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
