import os.path as osp
import rospkg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import MainWidget

from .urdf_parser import URDFParser
from .package_generator import PackageGenerator
from .robot_visualizer import RobotVisualizerWidget
from .settings import SettingsWidget
from .common import *


class SetupAssistant(MainWidget):
    def __init__(self) -> None:
        super().__init__(PKG_NAME, DEFAULT)

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        self.urdf_parser = URDFParser(self)
        self.pkg_generator = PackageGenerator(self)
        self.signals = Signals()

        rows = QVBoxLayout()
        self.setLayout(rows)

        # 高さを指定するために，単なる横並びのレイアウトもウィジェットとして定義している
        self.robot_visualizer = RobotVisualizerWidget(self)
        rows.addWidget(self.robot_visualizer)

        self.settings = SettingsWidget(self)
        rows.addWidget(self.settings)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        self.urdf_parser.define_connections()
        self.pkg_generator.define_connections()
        self.robot_visualizer.define_connections()
        self.settings.define_connections()

        # パッケージの作成が完了したら閉じる
        self.pkg_generator.generated.connect(self.close)
