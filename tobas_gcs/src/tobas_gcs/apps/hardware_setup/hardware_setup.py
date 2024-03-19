from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from pyqt_vertical_tab_widget.verticalTabWidget import VerticalTabWidget

from .tabs import *


class HardwareSetupWidget(VerticalTabWidget):
    NAME = "Hardware Setup"

    TAB_HEIGHT = 35  # これ以上無いと何故かTabBarの文字が横に見切れてしまう
    TAB_WIDTH = 80
    MIN_HEIGHT = 300

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
        self._main = main

        self._network_setting = NetworkSettingWidget(main)
        self._acc_calib = AccelCalibrationWidget(main)
        self._mag_calib = MagCalibrationWidget(main)
        self._adc_calib = AdcCalibrationWidget(main)
        self._rcin_calib = RcinCalibrationWidget(main)
        self._noise_calib = NoiseCalibrationWidget(main)
        self._esc_calib = EscCalibrationWidget(main)
        self._motor_test = MotorTestWidget(main)

        self.addTab(self._network_setting, NetworkSettingWidget.NAME)
        self.addTab(self._acc_calib, AccelCalibrationWidget.NAME)
        self.addTab(self._mag_calib, MagCalibrationWidget.NAME)
        self.addTab(self._adc_calib, AdcCalibrationWidget.NAME)
        self.addTab(self._rcin_calib, RcinCalibrationWidget.NAME)
        # self.addTab(self._noise_calib, NoiseCalibrationWidget.NAME)  # TODO
        self.addTab(self._esc_calib, EscCalibrationWidget.NAME)
        self.addTab(self._motor_test, MotorTestWidget.NAME)

        self.setMinimumHeight(self.MIN_HEIGHT)
        self.setStyleSheet(f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}")

    def define_connections(self) -> None:
        self._network_setting.define_connections()
        self._acc_calib.define_connections()
        self._mag_calib.define_connections()
        self._adc_calib.define_connections()
        self._rcin_calib.define_connections()
        self._noise_calib.define_connections()
        self._esc_calib.define_connections()
        self._motor_test.define_connections()

    def switch_to_tab(self, tab: QWidget) -> None:
        idx = self.indexOf(tab)
        assert idx >= 0
        self.setCurrentIndex(idx)
