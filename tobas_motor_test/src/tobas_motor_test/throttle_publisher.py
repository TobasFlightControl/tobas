import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from typing import List
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QGridLayout

from tobas_rqt_tools.widgets import Widget, FloatSliderDisplay
from tobas_tools_py.constants import Topic, MIN_THROTTLE, MAX_THROTTLE
from tobas_msgs.msg import Throttle, ThrottleArray

from .common import BUTTON_HEIGHT, COMMAND_PERIOD


class ThrottlePublisherWidget(Widget):
    CHANNEL_SIZE = 14
    MAX_ROWS = CHANNEL_SIZE // 2

    def __init__(self) -> None:
        super().__init__()

        rows = QVBoxLayout()
        self.setLayout(rows)

        grid = QGridLayout()
        rows.addLayout(grid)

        self._commanders: List[FloatSliderDisplay] = []
        for channel in range(self.CHANNEL_SIZE):
            commander = FloatSliderDisplay(self)
            commander.set_text(f"CH{channel}")
            commander.set_minimum(MIN_THROTTLE)
            commander.set_maximum(MAX_THROTTLE)
            commander.set_value(MIN_THROTTLE)
            commander.value_changed.connect(self._on_value_changed)
            self._commanders.append(commander)
            grid.addWidget(commander, channel % self.MAX_ROWS, channel // self.MAX_ROWS)

        self._minimum_button = QPushButton("Minimum")
        self._minimum_button.clicked.connect(self._on_minimum_button_clicked)
        self._minimum_button.setFixedHeight(BUTTON_HEIGHT)
        rows.addWidget(self._minimum_button)

        rows.addStretch()

        self._throttles_pub = rclpy.Publisher(Topic.Command.THROTTLES, ThrottleArray, queue_size=1)

        # モータが停止しないよう一定周期でコマンドを発行し続ける
        rclpy.Timer(Duration(COMMAND_PERIOD), self._command_timer_cb)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self._publish_current_values()

    @pyqtSlot()
    def _on_minimum_button_clicked(self) -> None:
        self._set_all_values(MIN_THROTTLE)
        self._publish_current_values()

    def _publish_current_values(self) -> None:
        throttles = ThrottleArray()
        throttles.header.stamp = Time.now()

        for channel, commander in enumerate(self._commanders):
            throttles.throttles.append(Throttle(channel, commander.get_value()))

        self._throttles_pub.publish(throttles)

    def _set_all_values(self, value: int) -> None:
        for commander in self._commanders:
            commander.blockSignals(True)
            commander.set_value(value)
            commander.blockSignals(False)

    def _command_timer_cb(self, _) -> None:
        self._publish_current_values()
