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
        settings = VerticalTabWidget()
        rows.addWidget(settings)

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

        settings.addTab(self.start, StartWidget.NAME)
        settings.addTab(self.battery, BatteryWidget.NAME)
        settings.addTab(self.propulsion_system, PropulsionSystemWidget.NAME)
        settings.addTab(self.fixed_wing, FixedWingWidget.NAME)
        settings.addTab(self.custom_joints, CustomJointsWidget.NAME)
        settings.addTab(self.imu, ImuWidget.NAME)
        settings.addTab(self.barometer, BarometerWidget.NAME)
        settings.addTab(self.gps, GpsWidget.NAME)
        settings.addTab(self.rgb_camera, RgbCameraWidget.NAME)
        settings.addTab(self.depth_camera, DepthCameraWidget.NAME)
        settings.addTab(self.lidar, LidarWidget.NAME)
        settings.addTab(self.odometry, OdometryWidget.NAME)
        settings.addTab(self.tether_station, TetherStationWidget.NAME)
        settings.addTab(self.controller, ControllerWidget.NAME)
        settings.addTab(self.observer, ObserverWidget.NAME)
        settings.addTab(self.simulation, SimulationWidget.NAME)
        settings.addTab(self.author_information, AuthorInformationWidget.NAME)
        settings.addTab(self.ros_package, RosPackageWidget.NAME)

        settings.setMinimumHeight(self.SETTINGS_MIN_HEIGHT)
        settings.setStyleSheet(f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}")
