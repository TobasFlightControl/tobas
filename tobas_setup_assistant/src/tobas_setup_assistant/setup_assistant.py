from PyQt5.QtWidgets import QVBoxLayout

from tobas_rqt_tools.widgets import Widget

from .urdf_parser import URDFParser
from .package_generator import PackageGenerator
from .robot_visualizer import RobotVisualizerWidget
from .settings import SettingsWidget
from .common import Signals


class SetupAssistant(Widget):
    def __init__(self) -> None:
        super().__init__()

        self.signals = Signals()

        self.urdf_parser = URDFParser(self)
        self.pkg_generator = PackageGenerator(self)

        rows = QVBoxLayout()
        self.setLayout(rows)

        # 高さを指定するために，単なる横並びのレイアウトもウィジェットとして定義している
        self.robot_visualizer = RobotVisualizerWidget(self)
        rows.addWidget(self.robot_visualizer)

        self.settings = SettingsWidget(self)
        rows.addWidget(self.settings)
