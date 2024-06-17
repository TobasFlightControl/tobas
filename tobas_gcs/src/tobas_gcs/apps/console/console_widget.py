from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtWidgets import QHeaderView, QTableWidgetItem, QVBoxLayout
from PyQt5.QtGui import QColor

from tobas_rqt_tools.widgets import TableWidget
from tobas_tools_py.constants import Topic
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Message

from ..base import BaseAppWidget


class ConsoleWidget(BaseAppWidget):
    NAME = "Console"

    MAX_ROWS = 1000  # 表示するメッセージの最大数

    LABELS = ("Stamp", "Name", "Level", "Message")
    COL_STAMP = 0
    COL_NAME = 1
    COL_LEVEL = 2
    COL_MESSAGE = 3

    # メッセージの色 (ケース不問)
    COLOR_DEBUG = "darkGreen"
    COLOR_INFO = "black"
    COLOR_WARN = "orange"
    COLOR_ERROR = "red"
    COLOR_FATAL = "blueViolet"
    COLOR_UNKNOWN = "darkGray"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._table = TableWidget(0, len(self.LABELS))
        self._table.setHorizontalHeaderLabels(self.LABELS)
        self._table.horizontalHeader().setSectionResizeMode(self.COL_STAMP, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(self.COL_NAME, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(self.COL_LEVEL, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(self.COL_MESSAGE, QHeaderView.Stretch)
        rows.addWidget(self._table)

        self._message_sub = None

    @override
    def update_internal_data_structures(self) -> None:
        self._table.remove_all()

        if self._message_sub is not None:
            self._message_sub.unregister()
        self._message_sub = rospy.Subscriber(
            f"{self._drone.name}/{Topic.MESSAGE}", Message, self._message_cb, queue_size=1
        )

    def _message_cb(self, message: Message) -> None:
        # TODO: ボタンを押すとデバッグメッセージを表示
        if message.level == Message.DEBUG:
            return

        # 先頭に行を追加
        self._table.insertRow(0)

        # 行が溢れていたら古い方から消す
        num_rows = self._table.rowCount()
        if num_rows > self.MAX_ROWS:
            self._table.removeRow(num_rows - 1)

        stamp = message.header.stamp
        stamp_item = QTableWidgetItem(f"{stamp.secs}.{stamp.nsecs}")
        self._table.setItem(0, self.COL_STAMP, stamp_item)

        name_item = QTableWidgetItem(message.name)
        self._table.setItem(0, self.COL_NAME, name_item)

        # TODO: メッセージにカーソルを重ねると全文を表示 (cf. rqt_console)
        message_item = QTableWidgetItem(message.message)

        if message.level == Message.DEBUG:
            raise NotImplementedError()
        elif message.level == Message.INFO:
            level_item = QTableWidgetItem("Info")
            level_item.setForeground(QColor(self.COLOR_INFO))
            message_item.setForeground(QColor(self.COLOR_INFO))
        elif message.level == Message.WARN:
            level_item = QTableWidgetItem("Warn")
            level_item.setForeground(QColor(self.COLOR_WARN))
            message_item.setForeground(QColor(self.COLOR_WARN))
        elif message.level == Message.ERROR:
            level_item = QTableWidgetItem("Error")
            level_item.setForeground(QColor(self.COLOR_ERROR))
            message_item.setForeground(QColor(self.COLOR_ERROR))
        elif message.level == Message.FATAL:
            level_item = QTableWidgetItem("Fatal")
            level_item.setForeground(QColor(self.COLOR_FATAL))
            message_item.setForeground(QColor(self.COLOR_FATAL))
        else:
            rospy.logerr(f"Unknown message level: {message.level}")
            level_item = QTableWidgetItem("Unknown")
            level_item.setForeground(QColor(self.COLOR_UNKNOWN))
            message_item.setForeground(QColor(self.COLOR_UNKNOWN))

        self._table.setItem(0, self.COL_LEVEL, level_item)
        self._table.setItem(0, self.COL_MESSAGE, message_item)
