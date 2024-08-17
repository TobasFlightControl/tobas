from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .base import BaseSelectedLinkSettingWidget

import rclpy
from rclpy.duration import Duration
import numpy as np
from numpy import linalg as LA
from typing import List
from typing import override
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from tobas_rqt_tools.widgets import TabWidget
from tobas_rqt_tools.messages import q_info, q_warn, q_error_named
from tobas_kdl_sympy.frames import Vector
from tobas_tools_py.rotor_config import RotorAxis

from ...common import PROP_TILT_TOL
from .common import PROPULSION_SYSTEM
from .esc import EscWidget
from .motor import MotorWidget
from .propeller import PropellerWidget
from .electrodynamics import ElectrodynamicsWidget
from .aerodynamics import AerodynamicsWidget


class SelectedLinksTabWidget(TabWidget):
    TAB_WIDTH = 150
    TAB_HEIGHT = 50
    ARROW_LENGTH = 0.2  # 想定される推力の最大値を矢印の長さに反映
    PUBLISH_MARKERS_TIMER_PERIOD = 100  # [ms]

    link_removed = pyqtSignal(str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.ignore_wheel_event()
        self.set_size(self.TAB_WIDTH, self.TAB_HEIGHT)
        self.setMovable(True)
        self.setTabsClosable(True)

        self._markers = MarkerArray()  # 推力の作用線
        self._markers_pub = self.create_publisher("visualization_marker_array", MarkerArray, 1)

        self._publish_markers_timer = QTimer(self)
        self._publish_markers_timer.timeout.connect(self._publish_markers_timer_cb)

        self.tabCloseRequested.connect(self._on_tab_close_requested)

    @override
    def clear(self) -> None:
        super().clear()
        self._markers.markers.clear()
        self._publish_markers_timer.stop()

    def update_internal_data_structures(self) -> None:
        self.clear()

        # 全ての可動リンクのマーカを保持しておく
        for i, link_name in enumerate(self._main.urdf_parser.movable_joint_names()):
            # ジョイントを取得
            joint = self._main.urdf_parser.get_joint(link_name)

            # 推力の作用線
            arrow_start = np.zeros((3,))
            arrow_end = np.array(joint.axis) / LA.norm(joint.axis) * self.ARROW_LENGTH
            arrow_scale = np.array([0.1, 0.2, 0.3]) * self.ARROW_LENGTH

            # マーカを作成
            marker = Marker()
            marker.header.frame_id = link_name
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.DELETE  # デフォルトでは非表示
            marker.points.append(Point(*arrow_start))
            marker.points.append(Point(*arrow_end))
            marker.scale = Vector3(*arrow_scale)
            marker.color = ColorRGBA(1.0, 0.4, 0.7, 1.0)  # TODO: 回転方向によって色分け
            marker.lifetime = Duration(0)  # 無限の生存期間
            marker.frame_locked = True  # TFが変化してもフレームに固定

            # マーカを追加
            self._markers.markers.append(marker)

        # マーカを発行開始
        self._publish_markers_timer.start(self.PUBLISH_MARKERS_TIMER_PERIOD)

    def is_valid(self) -> bool:
        num_rotors = self.count()

        # それぞれのタブの設定が有効であることを確認
        for i in range(num_rotors):
            tab: SelectedLinkWidget = self.widget(i)
            if not tab.is_valid():
                return False

        # 最低1つは登録されていなければならない
        if num_rotors == 0:
            q_error_named(
                self._main,
                PROPULSION_SYSTEM,
                "Please register at least 1 propulsion systems.",
            )
            return False

        return True

    def dump_settings(self, link_name: str) -> dict:
        return self._get_tab(link_name).dump_settings()

    def load_settings(self, link_name: str, data: dict) -> None:
        self._get_tab(link_name).load_settings(data)

    def get_index(self, link_name: str) -> int:
        """タブのインデックスを返す．"""
        for idx in range(self.count()):
            tab: SelectedLinkWidget = self.widget(idx)
            if tab.link_name() == link_name:
                return idx
        else:
            raise RuntimeError(f"Link name not found: {link_name}")

    def get_esc(self, link_name: str) -> EscWidget:
        return self._get_tab(link_name).esc

    def get_motor(self, link_name: str) -> MotorWidget:
        return self._get_tab(link_name).motor

    def get_propeller(self, link_name: str) -> PropellerWidget:
        return self._get_tab(link_name).propeller

    def get_electrodynamics(self, link_name: str) -> ElectrodynamicsWidget:
        return self._get_tab(link_name).electrodynamics

    def get_aerodynamics(self, link_name: str) -> AerodynamicsWidget:
        return self._get_tab(link_name).aerodynamics

    def link_name(self, idx: int) -> str:
        tab: SelectedLinkWidget = self.widget(idx)
        return tab.link_name()

    def link_names(self) -> List[str]:
        """選択テーブル内のリンクの名前のリストを返す．"""
        return [self.link_name(i) for i in range(self.count())]

    def joint_name(self, idx: int) -> str:
        tab: SelectedLinkWidget = self.widget(idx)
        return tab.joint_name()

    def joint_names(self) -> List[str]:
        """選択テーブル内のジョイントの名前のリストを返す．"""
        return [self.joint_name(i) for i in range(self.count())]

    def direction(self, idx: int) -> str:
        """CW or CCW"""
        tab: SelectedLinkWidget = self.widget(idx)
        return tab.motor.direction()

    def directions(self) -> List[str]:
        """選択テーブル内の回転方向 (CW or CCW) のリストを返す．"""
        return [self.direction(i) for i in range(self.count())]

    def add_link(self, link_name: str) -> None:
        # タブを追加
        tab = SelectedLinkWidget(self._main, link_name)
        self.addTab(tab, link_name)

        # 指定リンクのマーカを表示
        self._set_action(link_name, Marker.ADD)

    def remove_link(self, link_name: str) -> None:
        # タブを削除
        self.removeTab(self.get_index(link_name))

        # 指定リンクのマーカを非表示
        self._set_action(link_name, Marker.DELETE)

    def _get_tab(self, link_name: str) -> SelectedLinkWidget:
        idx = self.get_index(link_name)
        return self.widget(idx)

    def _set_action(self, link_name: str, action: int) -> None:
        """指定されたリンクのマーカのアクションを設定する．"""
        for i in range(0, len(self._markers.markers)):
            marker: Marker = self._markers.markers[i]
            if marker.header.frame_id == link_name:
                marker.action = action
                return

    def _publish_markers_timer_cb(self) -> None:
        self._markers_pub.publish(self._markers)

    @pyqtSlot(int)
    def _on_tab_close_requested(self, idx: int) -> None:
        link_name = self.link_name(idx)
        self.remove_link(link_name)
        self.link_removed.emit(link_name)


class SelectedLinkWidget(QWidget):
    BUTTON_WIDTH = 120
    BUTTON_HEIGHT = 50
    TAB_WIDTH = 135
    TAB_HEIGHT = 45

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        rows = QVBoxLayout()
        self.setLayout(rows)

        button_cols = QHBoxLayout()
        rows.addLayout(button_cols)

        self._copy_from_left_button = QPushButton("Copy From Left")
        self._copy_from_left_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._copy_from_left_button.clicked.connect(self._on_copy_from_left_button_clicked)
        button_cols.addWidget(self._copy_from_left_button)

        self._copy_to_all_button = QPushButton("Copy To All")
        self._copy_to_all_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._copy_to_all_button.clicked.connect(self._on_copy_to_all_button_clicked)
        button_cols.addWidget(self._copy_to_all_button)

        button_cols.addStretch()

        self._tab_widget = TabWidget()
        self._tab_widget.ignore_wheel_event()
        self._tab_widget.set_size(self.TAB_WIDTH, self.TAB_HEIGHT)
        rows.addWidget(self._tab_widget)

        self.esc = EscWidget(main, link_name)
        self.motor = MotorWidget(main, link_name)
        self.propeller = PropellerWidget(main, link_name)
        self.electrodynamics = ElectrodynamicsWidget(main, link_name)
        self.aerodynamics = AerodynamicsWidget(main, link_name)

        self._tab_widget.addTab(self.esc, EscWidget.NAME)
        self._tab_widget.addTab(self.motor, MotorWidget.NAME)
        self._tab_widget.addTab(self.propeller, PropellerWidget.NAME)
        self._tab_widget.addTab(self.electrodynamics, ElectrodynamicsWidget.NAME)
        self._tab_widget.addTab(self.aerodynamics, AerodynamicsWidget.NAME)

        rows.addStretch()

    def is_valid(self) -> bool:
        for i in range(self._tab_widget.count()):
            widget: BaseSelectedLinkSettingWidget = self._tab_widget.widget(i)
            if not widget.is_valid():
                return False

        return True

    def copy_from(self, src: SelectedLinkWidget) -> None:
        for i in range(self._tab_widget.count()):
            des_setting: BaseSelectedLinkSettingWidget = self._tab_widget.widget(i)
            src_setting: BaseSelectedLinkSettingWidget = src._tab_widget.widget(i)
            des_setting.copy_from(src_setting)

    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._tab_widget.count()):
            widget: BaseSelectedLinkSettingWidget = self._tab_widget.widget(i)
            res[widget.NAME] = widget.dump_settings()
        return res

    def load_settings(self, data: dict) -> None:
        for i in range(self._tab_widget.count()):
            widget: BaseSelectedLinkSettingWidget = self._tab_widget.widget(i)
            widget.load_settings(data[widget.NAME])

    def link_name(self) -> str:
        return self._link_name

    def joint_name(self) -> str:
        return self._main.urdf_parser.get_joint(self._link_name).name

    def axis_type(self) -> str:
        axis = self._main.urdf_parser.global_axis(self.joint_name())
        if axis.is_collinear(Vector.UnitX(), PROP_TILT_TOL):
            return RotorAxis.X_POSITIVE.name
        elif axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
            return RotorAxis.Z_POSITIVE.name
        else:
            return RotorAxis.UNKNOWN.name

    @pyqtSlot()
    def _on_copy_from_left_button_clicked(self) -> None:
        selected = self._main.propulsion_system.selected
        self_idx = selected.get_index(self._link_name)

        if self_idx == 0:
            q_warn(self._main, "There are no tabs on the left side.")
            return

        left: SelectedLinkWidget = selected.widget(self_idx - 1)
        self.copy_from(left)

        q_info(
            self._main,
            f"The settings from {left.link_name()} have been copied to {self.link_name()}.",
        )

    @pyqtSlot()
    def _on_copy_to_all_button_clicked(self) -> None:
        selected = self._main.propulsion_system.selected
        self_idx = selected.get_index(self._link_name)

        for i in range(selected.count()):
            if i == self_idx:
                continue
            des: SelectedLinkWidget = selected.widget(i)
            des.copy_from(self)

        q_info(
            self._main,
            f"The settings from {self.link_name()} have been copied to all the selected links.",
        )
