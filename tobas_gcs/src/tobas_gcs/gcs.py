import os.path as osp
import rospkg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import MainWidget, ComboBox, add_spacer

from .common import *
from .apps import *
from .package_loader import PackageLoaderWidget


class GroundControlStationWidget(MainWidget):
    def __init__(self) -> None:
        super().__init__(PKG_NAME, DEFAULT)

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        self.signals = Signals()

        self.start = StartWidget(self)
        self.urdf_builder = UrdfBuilderWidget(self)
        self.setup_assistant = SetupAssistantWidget(self)
        self.simulation = SimulationWidget(self)
        self.hardware_setup = HardwareSetupWidget(self)
        self.mission_planner = MissionPlannerWidget(self)
        self.control_system = ControlSystemWidget(self)

        combo_box = ComboBox()
        apps = QStackedWidget()
        for app in [
            self.start,
            self.urdf_builder,
            self.setup_assistant,
            self.simulation,
            self.hardware_setup,
            self.mission_planner,
            self.control_system,
        ]:
            combo_box.addItem(app.NAME)
            apps.addWidget(app)

        self.package_loader = PackageLoaderWidget(self)

        # レイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)
        cols = QHBoxLayout()
        rows.addLayout(cols)
        cols.addWidget(combo_box)
        add_spacer(cols)  # 横に拡大するスペーサを追加
        cols.addWidget(self.package_loader)
        rows.addWidget(apps)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        self.start.define_connections()
        self.urdf_builder.define_connections()
        self.setup_assistant.define_connections()
        self.simulation.define_connections()
        self.hardware_setup.define_connections()
        self.mission_planner.define_connections()
        self.control_system.define_connections()

        self.package_loader.define_connections()
