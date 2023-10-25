import os
import os.path as osp
from overrides import overrides
from configparser import ConfigParser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.path import get_proj_path

from .urdf_parser import URDFParser
from .package_generator import PackageGenerator
from .robot_visualizer import RobotVisualizerWidget
from .settings import SettingsWidget
from .common import *


class SetupAssistant(QWidget):
    POS_X_KEY = "main_window/pos_x"
    POS_Y_KEY = "main_window/pos_y"
    WIDTH_KEY = "main_window/width"
    HEIGHT_KEY = "main_window/height"

    def __init__(self) -> None:
        super().__init__()

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/tobas_icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        self.urdf_parser = URDFParser(self)
        self.pkg_generator = PackageGenerator(self)
        self.signals = Signals()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        # 高さを指定するために，単なる横並びのレイアウトもウィジェットとして定義している
        self.robot_visualizer = RobotVisualizerWidget(self)
        self._rows.addWidget(self.robot_visualizer)

        self.settings = SettingsWidget(self)
        self._rows.addWidget(self.settings)

        # configがなければ作成
        config_dir = osp.dirname(CONFIG_PATH)
        os.makedirs(config_dir, exist_ok=True)

        # 最新のウィンドウの位置とサイズを反映
        self._config = ConfigParser()
        self._config.read(CONFIG_PATH)
        pos_x = self._config.getint(DEFAULT, self.POS_X_KEY, fallback=-1)
        pos_y = self._config.getint(DEFAULT, self.POS_Y_KEY, fallback=-1)
        width = self._config.getint(DEFAULT, self.WIDTH_KEY, fallback=-1)
        height = self._config.getint(DEFAULT, self.HEIGHT_KEY, fallback=-1)
        if pos_x >= 0 and pos_y >= 0 and width > 0 and height > 0:
            self.setGeometry(pos_x, pos_y, width, height)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        self.urdf_parser.define_connections()
        self.pkg_generator.define_connections()
        self.robot_visualizer.define_connections()
        self.settings.define_connections()

        # パッケージの作成が完了したら閉じる
        self.pkg_generator.generated.connect(self.close)

    @overrides
    def moveEvent(self, event: QMoveEvent) -> None:
        # 現在のウィンドウ位置を保存
        self._config.read(CONFIG_PATH)
        cur_pos = self.pos()
        self._config[DEFAULT][self.POS_X_KEY] = str(cur_pos.x())
        self._config[DEFAULT][self.POS_Y_KEY] = str(cur_pos.y())
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

        return super().moveEvent(event)

    @overrides
    def resizeEvent(self, event: QResizeEvent) -> None:
        # 現在のウィンドウサイズを保存
        self._config.read(CONFIG_PATH)
        cur_size = self.size()
        self._config[DEFAULT][self.WIDTH_KEY] = str(cur_size.width())
        self._config[DEFAULT][self.HEIGHT_KEY] = str(cur_size.height())
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

        return super().resizeEvent(event)
