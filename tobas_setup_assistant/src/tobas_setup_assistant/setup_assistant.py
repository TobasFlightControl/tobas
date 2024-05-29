from PyQt5.QtWidgets import QVBoxLayout

from tobas_rqt_tools.widgets import Widget, VerticalTabWidget

from .urdf_parser import URDFParser
from .package_generator import PackageGenerator
from .robot_visualizer import RobotVisualizerWidget
from .common import Signals
from .setting_widgets import *


class SetupAssistant(Widget):
    TAB_HEIGHT = 30  # 30以上無いと何故かTabBarの文字が横に見切れてしまう
    TAB_WIDTH = 70
    SETTINGS_MIN_HEIGHT = 300

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

        # 設定項目
        self._tab_widget = VerticalTabWidget()
        rows.addWidget(self._tab_widget)

        self.start = StartWidget(self)
        self.battery = BatteryWidget(self)
        self.propulsion_system = PropulsionSystemWidget(self)
        self.fixed_wing = FixedWingWidget(self)
        self.custom_joints = CustomJointsWidget(self)
        self.imu = ImuWidget(self)
        self.barometer = BarometerWidget(self)
        self.gps = GpsWidget(self)
        self.rgb_camera = RgbCameraWidget(self)
        self.depth_camera = DepthCameraWidget(self)
        self.lidar = LidarWidget(self)
        self.odometry = OdometryWidget(self)
        self.tether_station = TetherStationWidget(self)
        self.controller = ControllerWidget(self)
        self.observer = ObserverWidget(self)
        self.simulation = SimulationWidget(self)
        self.author_information = AuthorInformationWidget(self)
        self.ros_package = RosPackageWidget(self)

        self._tab_widget.addTab(self.start, StartWidget.NAME)
        self._tab_widget.addTab(self.battery, BatteryWidget.NAME)
        self._tab_widget.addTab(self.propulsion_system, PropulsionSystemWidget.NAME)
        self._tab_widget.addTab(self.fixed_wing, FixedWingWidget.NAME)
        self._tab_widget.addTab(self.custom_joints, CustomJointsWidget.NAME)
        self._tab_widget.addTab(self.imu, ImuWidget.NAME)
        self._tab_widget.addTab(self.barometer, BarometerWidget.NAME)
        self._tab_widget.addTab(self.gps, GpsWidget.NAME)
        self._tab_widget.addTab(self.rgb_camera, RgbCameraWidget.NAME)
        self._tab_widget.addTab(self.depth_camera, DepthCameraWidget.NAME)
        self._tab_widget.addTab(self.lidar, LidarWidget.NAME)
        self._tab_widget.addTab(self.odometry, OdometryWidget.NAME)
        self._tab_widget.addTab(self.tether_station, TetherStationWidget.NAME)
        self._tab_widget.addTab(self.controller, ControllerWidget.NAME)
        self._tab_widget.addTab(self.observer, ObserverWidget.NAME)
        self._tab_widget.addTab(self.simulation, SimulationWidget.NAME)
        self._tab_widget.addTab(self.author_information, AuthorInformationWidget.NAME)
        self._tab_widget.addTab(self.ros_package, RosPackageWidget.NAME)

        self._tab_widget.setMinimumHeight(self.SETTINGS_MIN_HEIGHT)
        self._tab_widget.setStyleSheet(f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}")

        # 最初はスタートタブ以外を無効化
        for i in range(1, self._tab_widget.count()):
            widget: BaseSettingWidget = self._tab_widget.widget(i)
            widget.setEnabled(False)

    def update_internal_data_structures(self) -> None:
        self.pkg_generator.update_internal_data_structures()
        self.robot_visualizer.update_internal_data_structures()

        for i in range(self._tab_widget.count()):
            widget: BaseSettingWidget = self._tab_widget.widget(i)
            widget.update_internal_data_structures()
            widget.setEnabled(True)
