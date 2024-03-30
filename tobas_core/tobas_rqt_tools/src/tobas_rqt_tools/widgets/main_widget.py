import os
import signal
import rospy
from overrides import override
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QIcon, QCloseEvent, QMoveEvent, QResizeEvent

from tobas_std_tools_py.config_parser import ConfigParserWrapper


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

    def __init__(self, config_path: str, section: str, title: str, icon_path: str, widget: QWidget) -> None:
        super().__init__()

        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(icon_path))

        # ウィジェットを配置
        rows = QVBoxLayout()
        self.setLayout(rows)
        self._widget = widget
        rows.addWidget(self._widget)

        # configを読み込み
        self._config = ConfigParserWrapper(config_path, section)

        # 最新のウィンドウの位置とサイズを反映
        pos_x = self._config.getint(self.POS_X_KEY, fallback=-1)
        pos_y = self._config.getint(self.POS_Y_KEY, fallback=-1)
        width = self._config.getint(self.WIDTH_KEY, fallback=-1)
        height = self._config.getint(self.HEIGHT_KEY, fallback=-1)
        if pos_x >= 0 and pos_y >= 0 and width > 0 and height > 0:
            self.setGeometry(pos_x, pos_y, width, height)

    @override
    def closeEvent(self, _: QCloseEvent) -> None:
        rospy.logdebug(f"{self.__class__.__name__}.closeEvent")

        self._widget.close()

        # メインウィンドウが閉じられる時にプロセスごと落とすことで，確実に終了させる．
        os.kill(os.getpid(), signal.SIGINT)

    @override
    def moveEvent(self, event: QMoveEvent) -> None:
        # 現在のウィンドウ位置を保存
        self._config.read()
        cur_pos = self.pos()
        self._config.set(self.POS_X_KEY, cur_pos.x())
        self._config.set(self.POS_Y_KEY, cur_pos.y())
        self._config.write()

        return super().moveEvent(event)

    @override
    def resizeEvent(self, event: QResizeEvent) -> None:
        # 現在のウィンドウサイズを保存
        self._config.read()
        cur_size = self.size()
        self._config.set(self.WIDTH_KEY, cur_size.width())
        self._config.set(self.HEIGHT_KEY, cur_size.height())
        self._config.write()

        return super().resizeEvent(event)
