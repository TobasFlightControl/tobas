from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import VerticalTabWidget

from .setting_widgets import *


class SettingsWidget(VerticalTabWidget):
    TAB_HEIGHT = 30  # 30以上無いと何故かTabBarの文字が横に見切れてしまう
    TAB_WIDTH = 70
    MIN_HEIGHT = 300

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.start = StartWidget(main)
        self.battery = BatteryWidget(main)
        self.propulsion_system = PropulsionSystemWidget(main)
        self.fixed_wing = FixedWingWidget(main)
        self.custom_joints = CustomJointsWidget(main)
        self.imu = ImuWidget(main)
        self.barometer = BarometerWidget(main)
        self.gps = GpsWidget(main)
        self.rgb_camera = RgbCameraWidget(main)
        self.depth_camera = DepthCameraWidget(main)
        self.lidar = LidarWidget(main)
        self.odometry = OdometryWidget(main)
        self.controller = ControllerWidget(main)
        self.observer = ObserverWidget(main)
        self.simulation = SimulationWidget(main)
        self.author_information = AuthorInformationWidget(main)
        self.ros_package = RosPackageWidget(main)

        self.addTab(self.start, StartWidget.NAME)
        self.addTab(self.battery, BatteryWidget.NAME)
        self.addTab(self.propulsion_system, PropulsionSystemWidget.NAME)
        self.addTab(self.fixed_wing, FixedWingWidget.NAME)
        self.addTab(self.custom_joints, CustomJointsWidget.NAME)
        self.addTab(self.imu, ImuWidget.NAME)
        self.addTab(self.barometer, BarometerWidget.NAME)
        self.addTab(self.gps, GpsWidget.NAME)
        self.addTab(self.rgb_camera, RgbCameraWidget.NAME)
        self.addTab(self.depth_camera, DepthCameraWidget.NAME)
        self.addTab(self.lidar, LidarWidget.NAME)
        self.addTab(self.odometry, OdometryWidget.NAME)
        self.addTab(self.controller, ControllerWidget.NAME)
        self.addTab(self.observer, ObserverWidget.NAME)
        self.addTab(self.simulation, SimulationWidget.NAME)
        self.addTab(self.author_information, AuthorInformationWidget.NAME)
        self.addTab(self.ros_package, RosPackageWidget.NAME)

        self.setMinimumHeight(self.MIN_HEIGHT)
        self.setStyleSheet(f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}")

    def define_connections(self) -> None:
        self.start.define_connections()
        self.battery.define_connections()
        self.propulsion_system.define_connections()
        self.fixed_wing.define_connections()
        self.custom_joints.define_connections()
        self.imu.define_connections()
        self.barometer.define_connections()
        self.gps.define_connections()
        self.rgb_camera.define_connections()
        self.depth_camera.define_connections()
        self.lidar.define_connections()
        self.odometry.define_connections()
        self.controller.define_connections()
        self.observer.define_connections()
        self.simulation.define_connections()
        self.author_information.define_connections()
        self.ros_package.define_connections()

    def switch_to_tab(self, tab: QWidget) -> None:
        self.switch(tab)
