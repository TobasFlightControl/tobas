import sys
import threading
import signal
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path
from PyQt5.QtWidgets import QApplication

from tobas_rqt_py.widgets import MainWidget
from tobas_rqt_py.utils import handle_unexpected_exception

from .base_pose_commander import BasePoseCommanderWidget
from .common import PKG_NAME


def main(args=None):
    # ノードを起動
    rclpy.init(args=args)
    node = Node("base_pose_commander")
    threading.Thread(target=lambda: rclpy.spin(node)).start()

    # GUIを表示
    app = QApplication(sys.argv)
    pkg_path = get_package_share_path(PKG_NAME)
    widget = BasePoseCommanderWidget(node)
    main_widget = MainWidget("Base Pose Commander", str(pkg_path / "images/icon.png"), widget)
    main_widget.show()

    # Ctrl+Cで即終了
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.excepthook = handle_unexpected_exception

    # アプリケーションの終了時に全てのノードを落とす
    result = app.exec()
    rclpy.shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
