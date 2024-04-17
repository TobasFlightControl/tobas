from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget, ComboBox
from tobas_tools_py.drone import Drone

from .apps import *
from .package_manager import PackageManagerWidget
from .shutdown_button import ShutdownButtonWidget


class GroundControlStationWidget(Widget):
    def __init__(self) -> None:
        super().__init__()

        self._drone = Drone()

        self._start = StartWidget(self, self._drone)
        self._urdf_builder = UrdfBuilderWidget(self, self._drone)
        self._setup_assistant = SetupAssistantWidget(self, self._drone)
        self._hardware_setup = HardwareSetupWidget(self, self._drone)
        self._simulation = SimulationWidget(self, self._drone)
        self._mission_planner = MissionPlannerWidget(self, self._drone)
        self._control_system = ControlSystemWidget(self, self._drone)
        self._console = ConsoleWidget(self, self._drone)

        self._combo_box = ComboBox()
        self._combo_box.addItem(StartWidget.NAME)
        self._combo_box.addItem(UrdfBuilderWidget.NAME)
        self._combo_box.addItem(SetupAssistantWidget.NAME)
        self._combo_box.addItem(HardwareSetupWidget.NAME)
        self._combo_box.addItem(SimulationWidget.NAME)
        # self._combo_box.addItem(MissionPlannerWidget.NAME)  # TODO
        self._combo_box.addItem(ControlSystemWidget.NAME)
        self._combo_box.addItem(ConsoleWidget.NAME)

        self._apps = QStackedWidget()
        self._apps.addWidget(self._start)
        self._apps.addWidget(self._urdf_builder)
        self._apps.addWidget(self._setup_assistant)
        self._apps.addWidget(self._hardware_setup)
        self._apps.addWidget(self._simulation)
        # self._apps.addWidget(self._mission_planner)  # TODO
        self._apps.addWidget(self._control_system)
        self._apps.addWidget(self._console)

        self._package_manager = PackageManagerWidget(self, self._drone)
        self._shutdown_button = ShutdownButtonWidget(self, self._drone)

        # レイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)
        cols = QHBoxLayout()
        rows.addLayout(cols)
        cols.addWidget(self._combo_box)
        cols.addWidget(self._package_manager)
        cols.addWidget(self._shutdown_button)
        cols.setAlignment(self._combo_box, Qt.AlignLeft)
        cols.setAlignment(self._package_manager, Qt.AlignCenter)
        cols.setAlignment(self._shutdown_button, Qt.AlignRight)
        rows.addWidget(self._apps)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        # 選択リストから選択されたアプリケーションを表示
        self._combo_box.currentIndexChanged.connect(self._apps.setCurrentIndex)

        self._start.define_connections()
        self._urdf_builder.define_connections()
        self._setup_assistant.define_connections()
        self._hardware_setup.define_connections()
        self._simulation.define_connections()
        self._mission_planner.define_connections()
        self._control_system.define_connections()
        self._console.define_connections()

    def update_internal_data_structures(self) -> None:
        """ドローンの更新に応じて内部データを更新．"""
        self._start.update_internal_data_structures()
        self._urdf_builder.update_internal_data_structures()
        self._setup_assistant.update_internal_data_structures()
        self._hardware_setup.update_internal_data_structures()
        self._simulation.update_internal_data_structures()
        self._mission_planner.update_internal_data_structures()
        self._control_system.update_internal_data_structures()
        self._console.update_internal_data_structures()

    def package_path(self) -> str:
        return self._package_manager.package_path()
