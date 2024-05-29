from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import rospy
import numpy as np
from numpy import linalg as LA
from typing import List
from overrides import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from tobas_rqt_tools.widgets import TabWidget
from tobas_rqt_tools.messages import q_error_named
from tobas_rqt_tools.utils import place_center
from tobas_kdl_sympy.frames import Vector

from ...common import PROP_TILT_TOL
from .common import PROPULSION_SYSTEM, AxisType
from .signals import PropulsionSystemSignals
from .esc import EscWidget
from .motor import MotorWidget
from .blade_geometry import BladeGeometry
from .aerodynamics import AerodynamicsWidget


class SelectedLinksWidget(TabWidget):
    TAB_HEIGHT = 50
    TAB_WIDTH = 150
    ARROW_LENGTH = 0.2  # 想定される推力の最大値を矢印の長さに反映

    def __init__(self, main: SetupAssistant, signals: PropulsionSystemSignals) -> None:
        super().__init__()
        self._main = main
        self._signals = signals

        self.setStyleSheet(f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}")
        self.setMovable(True)
        self.setTabsClosable(True)

        self._markers = MarkerArray()  # 推力の作用線
        self._markers_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=1)

        signals.add_link.connect(self._add_link)
        signals.remove_link.connect(self._remove_link)
        self.tabCloseRequested.connect(self._on_tab_close_requested)

    @override
    def clear(self) -> None:
        super().clear()
        self._markers.markers.clear()

    def update_internal_data_structures(self) -> None:
        self.clear()

        # 全ての可動リンクのマーカーを保持しておく
        for i, link_name in enumerate(self._main.urdf_parser.link_names()):
            if link_name == self._main.urdf_parser.get_root().name:
                continue

            # ジョイントを取得
            joint = self._main.urdf_parser.get_joint(link_name)
            if joint.axis is None:
                continue

            # 推力の作用線
            arrow_start = np.zeros((3,))
            arrow_end = np.array(joint.axis) / LA.norm(joint.axis) * self.ARROW_LENGTH
            arrow_scale = np.array([0.1, 0.2, 0.3]) * self.ARROW_LENGTH

            # マーカーを作成
            marker = Marker()
            marker.header.frame_id = link_name
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.DELETE  # デフォルトでは非表示
            marker.points.append(Point(*arrow_start))
            marker.points.append(Point(*arrow_end))
            marker.scale = Vector3(*arrow_scale)
            marker.color = ColorRGBA(1.0, 0.4, 0.7, 1.0)  # TODO: 回転方向によって色分け
            marker.lifetime = rospy.Duration(0)  # 無限の生存期間
            marker.frame_locked = True  # TFが変化してもフレームに固定

            # マーカーを追加
            self._markers.markers.append(marker)

    def is_valid(self) -> bool:
        num_rotors = self.count()

        # それぞれのタブの設定が有効であることを確認
        for i in range(num_rotors):
            tab: SelectedLinkTabWidget = self.widget(i)
            if not tab.is_valid():
                return False

        # 最低1つは登録されていなければならない
        if num_rotors == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Please register at least 1 propulsion systems.")
            return

        return True

    def get_index(self, link_name: str) -> int:
        """タブのインデックスを返す．"""
        for idx in range(self.count()):
            tab: SelectedLinkTabWidget = self.widget(idx)
            if tab.link_name() == link_name:
                return idx
        else:
            raise RuntimeError(f"Link name not found: {link_name}")

    def get_esc(self, link_name: str) -> EscWidget:
        return self._get_tab(link_name).esc

    def get_motor(self, link_name: str) -> MotorWidget:
        return self._get_tab(link_name).motor

    def get_blade_geometry(self, link_name: str) -> BladeGeometry:
        return self._get_tab(link_name).blade_geometry

    def get_aerodynamics(self, link_name: str) -> AerodynamicsWidget:
        return self._get_tab(link_name).aerodynamics

    def link_name(self, idx: int) -> str:
        tab: SelectedLinkTabWidget = self.widget(idx)
        return tab.link_name()

    def link_names(self) -> List[str]:
        """選択テーブル内のリンクの名前のリストを返す．"""
        return [self.link_name(i) for i in range(self.count())]

    def joint_name(self, idx: int) -> str:
        tab: SelectedLinkTabWidget = self.widget(idx)
        return tab.joint_name()

    def joint_names(self) -> List[str]:
        """選択テーブル内のジョイントの名前のリストを返す．"""
        return [self.joint_name(i) for i in range(self.count())]

    def direction(self, idx: int) -> str:
        """CW or CCW"""
        tab: SelectedLinkTabWidget = self.widget(idx)
        return tab.motor.direction()

    def directions(self) -> List[str]:
        """選択テーブル内の回転方向 (CW or CCW) のリストを返す．"""
        return [self.direction(i) for i in range(self.count())]

    def _get_tab(self, link_name: str) -> SelectedLinkTabWidget:
        idx = self.get_index(link_name)
        return self.widget(idx)

    def _set_action(self, link_name: str, action: int) -> None:
        """指定されたリンクのマーカーのアクションを設定する．"""
        for i in range(0, len(self._markers.markers)):
            marker: Marker = self._markers.markers[i]
            if marker.header.frame_id == link_name:
                marker.action = action
                return

    @pyqtSlot(str)
    def _add_link(self, link_name: str) -> None:
        # タブを追加
        tab = SelectedLinkTabWidget(self._main, link_name)
        self.addTab(tab, link_name)

        # 指定リンクのマーカーを表示
        self._set_action(link_name, Marker.ADD)

        # マーカーを発行
        self._markers_pub.publish(self._markers)

    @pyqtSlot(str)
    def _remove_link(self, link_name: str) -> None:
        # タブを削除
        self.removeTab(self.get_index(link_name))

        # 指定リンクのマーカーを非表示
        self._set_action(link_name, Marker.DELETE)

        # マーカーを発行
        self._markers_pub.publish(self._markers)

    @pyqtSlot(int)
    def _on_tab_close_requested(self, idx: int) -> None:
        self._signals.remove_link.emit(self.link_name(idx))
        self._main.signals.airframe_updated.emit()


class SelectedLinkTabWidget(QWidget):
    BUTTON_WIDTH = 150
    BUTTON_HEIGHT = 40

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._copy_button = QPushButton("Copy from left tab")
        self._copy_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        place_center(self._copy_button, rows)

        self.esc = EscWidget(main, link_name)
        rows.addWidget(self.esc)

        self.blade_geometry = BladeGeometry(main, link_name)
        rows.addWidget(self.blade_geometry)

        self.motor = MotorWidget(main, link_name)
        rows.addWidget(self.motor)

        self.aerodynamics = AerodynamicsWidget(main, link_name)
        rows.addWidget(self.aerodynamics)

        rows.addStretch()
        self._define_connections()

    def is_valid(self) -> bool:
        if not self.esc.is_valid():
            return False
        if not self.blade_geometry.is_valid():
            return False
        if not self.motor.is_valid():
            return False
        if not self.aerodynamics.is_valid():
            return False

        return True

    def link_name(self) -> str:
        return self._link_name

    def joint_name(self) -> str:
        return self._main.urdf_parser.get_joint(self._link_name).name

    def axis_type(self) -> str:
        axis = self._main.urdf_parser.global_axis(self.joint_name())
        if axis.is_collinear(Vector.UnitX(), PROP_TILT_TOL):
            return AxisType.X_POSITIVE
        elif axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
            return AxisType.Z_POSITIVE
        else:
            return AxisType.UNKNOWN

    def _define_connections(self) -> None:
        self._copy_button.clicked.connect(self._copy_from_left_tab)

    @pyqtSlot()
    def _copy_from_left_tab(self) -> None:
        selected = self._main.propulsion_system.selected
        self_idx = selected.get_index(self._link_name)

        if self_idx == 0:
            return

        left: SelectedLinkTabWidget = selected.widget(self_idx - 1)
        self.esc.copy_from(left.esc)
        self.blade_geometry.copy_from(left.blade_geometry)
        self.motor.copy_from(left.motor)
        self.aerodynamics.copy_from(left.aerodynamics)
