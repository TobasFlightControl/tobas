import os
import signal
import rospy
from overrides import override
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QIcon, QCloseEvent


class MainWidget(QWidget):
    """
    メイン画面
    - Configを作成
    - 最新のウィンドウ位置とサイズを保存
    """

    POS_X_KEY = "main_window/pos_x"
    POS_Y_KEY = "main_window/pos_y"
    WIDTH_KEY = "main_window/width"
    HEIGHT_KEY = "main_window/height"

    def __init__(self, title: str, icon_path: str, widget: QWidget) -> None:
        super().__init__()

        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(icon_path))

        rows = QVBoxLayout()
        self.setLayout(rows)
        self._widget = widget
        rows.addWidget(self._widget)

    @override
    def closeEvent(self, _: QCloseEvent) -> None:
        rospy.logdebug(f"{self.__class__.__name__}.closeEvent")

        self._widget.close()

        # クローズ時にプロセスごと落とすことで確実に終了させる
        os.kill(os.getpid(), signal.SIGINT)
