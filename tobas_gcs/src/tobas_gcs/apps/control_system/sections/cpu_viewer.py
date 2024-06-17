from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtWidgets import QLabel

from tobas_rqt_tools.widgets import FramedLabel
from tobas_rqt_tools.layouts import FormLayout
from tobas_tools_py.constants import Topic
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Cpu

from .base_section import BaseControlSystemSectionWidget


class CpuViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "CPU"

    BOX_WIDTH = 100

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        form = FormLayout()
        self._rows.addLayout(form)

        self._temperature = FramedLabel()
        self._temperature.setFixedWidth(self.BOX_WIDTH)
        form.addRow(QLabel("Temperature"), self._temperature)

        self._frequency = FramedLabel()
        self._frequency.setFixedWidth(self.BOX_WIDTH)
        form.addRow(QLabel("Frequency"), self._frequency)

        self._load = FramedLabel()
        self._load.setFixedWidth(self.BOX_WIDTH)
        form.addRow(QLabel("Load"), self._load)

        self._cpu_sub = None

    @override
    def update_internal_data_structures(self) -> None:
        self._temperature.clear()
        self._frequency.clear()
        self._load.clear()

        if self._cpu_sub is not None:
            self._cpu_sub.unregister()
        self._cpu_sub = rospy.Subscriber(f"{self._drone.drone_name}/{Topic.CPU}", Cpu, self._cpu_cb, queue_size=1)

    def _cpu_cb(self, cpu: Cpu) -> None:
        # Temperature
        self._temperature.setText(f"{cpu.temperature:.1f} ℃")

        # Frequency
        if cpu.frequency < 1e9:
            freq_mhz = int(cpu.frequency * 1e-6)
            self._frequency.setText(f"{freq_mhz} MHz")
        else:
            freq_ghz = cpu.frequency * 1e-9
            self._frequency.setText(f"{freq_ghz:.1f} GHz")

        # Load
        cpu_load_percent = cpu.load * 100
        self._load.setText(f"{cpu_load_percent:.1f} %")
