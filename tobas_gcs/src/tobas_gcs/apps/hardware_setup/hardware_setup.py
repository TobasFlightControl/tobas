from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import VerticalTabWidget
from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget
from .tabs import *


class HardwareSetupWidget(BaseAppWidget):
    NAME = "Hardware Setup"

    TAB_HEIGHT = 35  # これ以上無いと何故かTabBarの文字が横に見切れてしまう
    TAB_WIDTH = 80
    MIN_HEIGHT = 300

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._tabs = VerticalTabWidget()
        rows.addWidget(self._tabs)

        self._network_setting = NetworkSettingWidget(main, drone)
        self._acc_calib = AccelCalibrationWidget(main, drone)
        self._mag_calib = MagCalibrationWidget(main, drone)
        self._adc_calib = AdcCalibrationWidget(main, drone)
        self._rcin_calib = RcinCalibrationWidget(main, drone)
        # self._noise_calib = NoiseCalibrationWidget(main, drone)  # TODO
        self._esc_calib = EscCalibrationWidget(main, drone)
        self._motor_test = MotorTestWidget(main, drone)

        self._tabs.addTab(self._network_setting, NetworkSettingWidget.NAME)
        self._tabs.addTab(self._acc_calib, AccelCalibrationWidget.NAME)
        self._tabs.addTab(self._mag_calib, MagCalibrationWidget.NAME)
        self._tabs.addTab(self._adc_calib, AdcCalibrationWidget.NAME)
        self._tabs.addTab(self._rcin_calib, RcinCalibrationWidget.NAME)
        # self._tabs.addTab(self._noise_calib, NoiseCalibrationWidget.NAME)  # TODO
        self._tabs.addTab(self._esc_calib, EscCalibrationWidget.NAME)
        self._tabs.addTab(self._motor_test, MotorTestWidget.NAME)

        self._tabs.setMinimumHeight(self.MIN_HEIGHT)
        self._tabs.setStyleSheet(f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}")

    @override
    def define_connections(self) -> None:
        self._network_setting.define_connections()
        self._acc_calib.define_connections()
        self._mag_calib.define_connections()
        self._adc_calib.define_connections()
        self._rcin_calib.define_connections()
        # self._noise_calib.define_connections()  # TODO
        self._esc_calib.define_connections()
        self._motor_test.define_connections()

    @override
    def update_internal_data_structures(self) -> None:
        self._network_setting.update_internal_data_structures()
        self._acc_calib.update_internal_data_structures()
        self._mag_calib.update_internal_data_structures()
        self._adc_calib.update_internal_data_structures()
        self._rcin_calib.update_internal_data_structures()
        # self._noise_calib.update_internal_data_structures()  # TODO
        self._esc_calib.update_internal_data_structures()
        self._motor_test.update_internal_data_structures()
