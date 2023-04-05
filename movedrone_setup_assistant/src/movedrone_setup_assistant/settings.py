from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from pyqt_vertical_tab_widget.verticalTabWidget import VerticalTabWidget
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .setting_widgets import *


class SettingsWidget(VerticalTabWidget):

    TAB_HEIGHT = 30  # 30以上無いと何故かTabBarの文字が横に見切れてしまう
    TAB_WIDTH = 70
    MIN_HEIGHT = 600

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.start = StartWidget(main)
        self.propellers = PropellersWidget(main)
        self.battery = BatteryWidget(main)
        self.imu = ImuWidget(main)
        self.magnetometer = MagnetometerWidget(main)
        self.barometer = BarometerWidget(main)
        self.gps = GpsWidget(main)
        self.rgb_camera = RgbCameraWidget(main)
        self.depth_camera = DepthCameraWidget(main)
        self.lidar = LidarWidget(main)
        self.controller = ControllerWidget(main)
        self.simulation = SimulationWidget(main)
        self.author_information = AuthorInformationWidget(main)
        self.ros_package = RosPackageWidget(main)

        self.addTab(self.start, "Start")
        self.addTab(self.propellers, "Propellers")
        self.addTab(self.battery, "Battery")
        self.addTab(self.imu, "IMU")
        self.addTab(self.magnetometer, "Magnetic")
        self.addTab(self.barometer, "Barometer")
        self.addTab(self.gps, "GPS")
        self.addTab(self.rgb_camera, "RGB Camera")
        self.addTab(self.depth_camera, "Depth Camera")
        # self.addTab(self.lidar, "LiDAR")  # TODO
        self.addTab(self.controller, "Controller")
        self.addTab(self.simulation, "Simulation")
        self.addTab(self.author_information, "Author Info")
        self.addTab(self.ros_package, "ROS Package")

        self.setMinimumHeight(self.MIN_HEIGHT)
        self.setStyleSheet(
            f'QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}'
        )

    def define_connections(self) -> None:
        self.start.define_connections()
        self.propellers.define_connections()
        self.battery.define_connections()
        self.imu.define_connections()
        self.magnetometer.define_connections()
        self.barometer.define_connections()
        self.gps.define_connections()
        self.rgb_camera.define_connections()
        self.depth_camera.define_connections()
        self.lidar.define_connections()
        self.controller.define_connections()
        self.simulation.define_connections()
        self.author_information.define_connections()
        self.ros_package.define_connections()
