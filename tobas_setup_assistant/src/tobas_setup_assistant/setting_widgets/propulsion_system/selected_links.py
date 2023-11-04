from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import TabWidget, add_expanding_widget, add_center_button
from dh_rqt_tools.messages import q_error_named
from kdl_sympy.frames import Vector

from ...parameter_getters import *
from ...common import *
from .common import ROTARY_WINGS, AxisType
from .esc import EscWidget
from .motor import MotorWidget
from .blade_geometry import BladeGeometry
from .aerodynamics import AerodynamicsWidget


class SelectedLinksWidget(TabWidget):
    TAB_HEIGHT = 50
    TAB_WIDTH = 150

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self._main = main

        self.setStyleSheet(
            f"QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}"
        )
        self.setMovable(True)
        self.setTabsClosable(True)

    def define_connections(self):
        self.tabCloseRequested.connect(self._on_tab_close_requested)

    def is_valid(self) -> bool:
        num_rotors = self.count()

        # それぞれのタブの設定が有効であることを確認
        for i in range(num_rotors):
            tab: SelectedLinkTabWidget = self.widget(i)
            if not tab.is_valid():
                return False

        # 最低1つは登録されていなければならない
        if num_rotors == 0:
            q_error_named(
                self._main,
                ROTARY_WINGS,
                "Please register at least 1 propulsion systems.",
            )
            return

        return True

    def add(self, link_name: str) -> None:
        tab = SelectedLinkTabWidget(self._main, link_name)
        self.addTab(tab, link_name)

    def get_index(self, link_name: str) -> int:
        """タブのインデックスを返す．"""
        for idx in range(self.count()):
            tab: SelectedLinkTabWidget = self.widget(idx)
            if tab.link_name() == link_name:
                return idx
        else:
            raise RuntimeError(f"Link name not found: {link_name}")

    def get_tab(self, link_name: str) -> SelectedLinkTabWidget:
        idx = self.get_index(link_name)
        return self.widget(idx)

    def get_esc(self, link_name: str) -> EscWidget:
        return self.get_tab(link_name).esc

    def get_motor(self, link_name: str) -> MotorWidget:
        return self.get_tab(link_name).motor

    def get_blade_geometry(self, link_name: str) -> BladeGeometry:
        return self.get_tab(link_name).blade_geometry

    def get_aerodynamics(self, link_name: str) -> AerodynamicsWidget:
        return self.get_tab(link_name).aerodynamics

    def link_name(self, rotor_idx: int) -> str:
        tab: SelectedLinkTabWidget = self.widget(rotor_idx)
        return tab.link_name()

    def link_names(self) -> List[str]:
        """選択テーブル内のリンクの名前のリストを返す．"""
        return [self.link_name(i) for i in range(self.count())]

    def joint_name(self, rotor_idx: int) -> str:
        tab: SelectedLinkTabWidget = self.widget(rotor_idx)
        return tab.joint_name()

    def joint_names(self) -> List[str]:
        """選択テーブル内のジョイントの名前のリストを返す．"""
        return [self.joint_name(i) for i in range(self.count())]

    def direction(self, rotor_idx: int) -> str:
        """CW or CCW"""
        tab: SelectedLinkTabWidget = self.widget(rotor_idx)
        return tab.motor.direction()

    def directions(self) -> List[str]:
        """選択テーブル内の回転方向 (CW or CCW) のリストを返す．"""
        return [self.direction(i) for i in range(self.count())]

    @pyqtSlot(int)
    def _on_tab_close_requested(self, idx: int) -> None:
        tab: SelectedLinkTabWidget = self.widget(idx)
        self._main.settings.propulsion_system.available.add(tab.link_name())
        self.removeTab(idx)

        self._main.signals.airframe_updated.emit()


class SelectedLinkTabWidget(QWidget):
    CP_BUTTON_HEIGHT = 40
    CP_BUTTON_WIDTH = 150
    RM_BUTTON_HEIGHT = 40
    RM_BUTTON_WIDTH = 100

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self.copy_button = add_center_button("Copy from left tab", self._rows)
        self.copy_button.setFixedSize(
            QSize(self.CP_BUTTON_WIDTH, self.CP_BUTTON_HEIGHT)
        )

        self.esc = EscWidget(main, link_name)
        self._rows.addWidget(self.esc)

        self.motor = MotorWidget(main, link_name)
        self._rows.addWidget(self.motor)

        self.blade_geometry = BladeGeometry(main, link_name)
        self._rows.addWidget(self.blade_geometry)

        self.aerodynamics = AerodynamicsWidget(main, link_name)
        self._rows.addWidget(self.aerodynamics)

        add_expanding_widget(self._rows)
        self._define_connections()

    def is_valid(self) -> bool:
        if not self.esc.is_valid():
            return False
        if not self.motor.is_valid():
            return False
        if not self.blade_geometry.is_valid():
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
            return UNKNOWN  # TODO

    def _define_connections(self) -> None:
        self.copy_button.clicked.connect(self._copy_from_left_tab)

    @pyqtSlot()
    def _copy_from_left_tab(self) -> None:
        selected = self._main.settings.propulsion_system.selected
        self_idx = selected.get_index(self._link_name)

        if self_idx == 0:
            return

        left: SelectedLinkTabWidget = selected.widget(self_idx - 1)
        self.esc.copy_from(left.esc)
        self.motor.copy_from(left.motor)
        self.blade_geometry.copy_from(left.blade_geometry)
        self.aerodynamics.copy_from(left.aerodynamics)
