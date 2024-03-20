from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import ComboBox

from .common import *
from .apps import *
from .package_manager import PackageManagerWidget


class GroundControlStationWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.signals = Signals()

        self.start = StartWidget(self)
        self.urdf_builder = UrdfBuilderWidget(self)
        self.setup_assistant = SetupAssistantWidget(self)
        self.simulation = SimulationWidget(self)
        self.hardware_setup = HardwareSetupWidget(self)
        self.mission_planner = MissionPlannerWidget(self)
        self.control_system = ControlSystemWidget(self)

        self._combo_box = ComboBox()
        self._combo_box.addItem(StartWidget.NAME)
        self._combo_box.addItem(UrdfBuilderWidget.NAME)
        self._combo_box.addItem(SetupAssistantWidget.NAME)
        self._combo_box.addItem(SimulationWidget.NAME)
        self._combo_box.addItem(HardwareSetupWidget.NAME)
        self._combo_box.addItem(MissionPlannerWidget.NAME)
        self._combo_box.addItem(ControlSystemWidget.NAME)

        self._apps = QStackedWidget()
        self._apps.addWidget(self.start)
        self._apps.addWidget(self.urdf_builder)
        self._apps.addWidget(self.setup_assistant)
        self._apps.addWidget(self.simulation)
        self._apps.addWidget(self.hardware_setup)
        self._apps.addWidget(self.mission_planner)
        self._apps.addWidget(self.control_system)

        self.package_manager = PackageManagerWidget(self)

        # レイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)
        cols = QHBoxLayout()
        rows.addLayout(cols)
        cols.addWidget(self._combo_box)
        cols.addStretch()
        cols.addWidget(self.package_manager)
        rows.addWidget(self._apps)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        # コンボボックスで選択されたアプリケーションを表示
        self._combo_box.currentIndexChanged.connect(self._apps.setCurrentIndex)

        self.start.define_connections()
        self.urdf_builder.define_connections()
        self.setup_assistant.define_connections()
        self.simulation.define_connections()
        self.hardware_setup.define_connections()
        self.mission_planner.define_connections()
        self.control_system.define_connections()

        self.package_manager.define_connections()
