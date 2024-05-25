from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtWidgets import QLabel

from tobas_std_tools_py.math import remap
from tobas_rqt_tools.widgets import HPositionBarWidget
from tobas_rqt_tools.layouts import FormLayout
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Battery

from .base_section import BaseControlSystemSectionWidget


class BatteryViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Battery"

    RANGE_WIDTH = 500
    RANGE_HEIGHT = 30

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        form = FormLayout()
        self._rows.addLayout(form)

        self._voltage_range = HPositionBarWidget()
        self._voltage_range.setFixedSize(self.RANGE_WIDTH, self.RANGE_HEIGHT)
        form.addRow(QLabel("Voltage"), self._voltage_range)

        self._current_range = HPositionBarWidget(fill_range=False)
        self._current_range.setFixedSize(self.RANGE_WIDTH, self.RANGE_HEIGHT)
        form.addRow(QLabel("Current"), self._current_range)

        self._battery_sub = None

    @override
    def update_internal_data_structures(self) -> None:
        self._voltage_range.clear()
        self._voltage_range.set_lower(self._drone.battery.sag_voltage)
        self._voltage_range.set_minimum(self._drone.battery.sag_voltage)
        self._voltage_range.set_maximum(self._drone.battery.max_voltage)
        self._voltage_range.start_timer()

        self._current_range.clear()
        self._current_range.set_minimum(0.0)
        self._current_range.set_maximum(self._drone.battery.max_current)
        self._current_range.start_timer()

        if self._battery_sub is not None:
            self._battery_sub.unregister()
        self._battery_sub = rospy.Subscriber(
            f"{self._drone.drone_name}/battery_filtered", Battery, self._battery_cb, queue_size=1
        )

    def _battery_cb(self, battery: Battery) -> None:
        rate = remap(battery.voltage, self._drone.battery.sag_voltage, self._drone.battery.max_voltage, 0, 100)
        self._voltage_range.set_upper(battery.voltage)
        self._voltage_range.set_text(f"{battery.voltage:.2f} V ({int(rate)} %)")

        self._current_range.set_value(battery.current)
        self._current_range.set_text(f"{battery.current:.2f} A")
