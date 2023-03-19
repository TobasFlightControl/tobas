import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.path import get_proj_path

from .urdf_parser import URDFParser
from .package_generator import PackageGenerator
from .robot_visualizer import RobotVisualizerWidget
from .settings import SettingsWidget
from .const import *


class SetupAssistant(QWidget):

    def __init__(self) -> None:
        super().__init__()

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/movedrone_icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        self.urdf_parser = URDFParser(self)
        self.pkg_generator = PackageGenerator(self)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        # 高さを指定するために，単なる横並びのレイアウトもウィジェットとして定義している
        self.robot_visualizer = RobotVisualizerWidget(self)
        self._rows.addWidget(self.robot_visualizer)

        self.settings = SettingsWidget(self)
        self._rows.addWidget(self.settings)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        self.urdf_parser.define_connections()
        self.pkg_generator.define_connections()
        self.robot_visualizer.define_connections()
        self.settings.define_connections()

        # パッケージの作成が完了したら閉じる
        self.pkg_generator.generated.connect(self.close)
