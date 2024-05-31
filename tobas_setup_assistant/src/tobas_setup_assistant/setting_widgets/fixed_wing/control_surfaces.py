from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import math
from enum import Enum
from typing import List, Union, Optional
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QPushButton, QListWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QFont

from tobas_std_tools_py.string import title_from_snake
from tobas_rqt_tools.widgets import DoubleSpinBox, TableWidget
from tobas_rqt_tools.messages import q_error
from tobas_kdl_sympy.joint import JointType

from ...common import LABEL_PSIZE, BODY_PSIZE, Signals
from .common import STABILITY_COEF_DECIMALS


class ControlSufraceField(Enum):
    LINK_NAME = 0
    JOINT_NAME = 1
    MIN_ANGLE = 2
    MAX_ANGLE = 3
    MAX_ANGLE_RATE = 4
    LIFT_COEF = 5
    DRAG_COEF = 6
    SIDE_COEF = 7
    ROLL_COEF = 8
    PITCH_COEF = 9
    YAW_COEF = 10


class ControlSurface:
    def __init__(self) -> None:
        self.joint_name = ""

        self.min_angle = 0.0
        self.max_angle = 0.0
        self.max_angle_rate = 0.0

        self.c_lift_delta = 0.0  # [/rad]
        self.c_drag_abs_delta = 0.0  # [/rad]
        self.c_side_delta = 0.0  # [/rad]
        self.c_roll_delta = 0.0  # [/rad]
        self.c_pitch_delta = 0.0  # [/rad]
        self.c_yaw_delta = 0.0  # [/rad]


class ControlSurfacesWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        available_links_label = QLabel("Available Links")
        available_links_label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        available_links_label.setAlignment(Qt.AlignLeft)
        rows.addWidget(available_links_label)

        self.available_links = AvailableLinksWidget(main)
        self.selected = SelectedLinksWidget(main)
        self.add_delete = AddDeleteButtonsWidget(self._main.signals, self.available_links, self.selected)

        rows.addWidget(self.available_links)
        rows.addWidget(self.add_delete)
        rows.addWidget(self.selected)

    def update_internal_data_structures(self) -> None:
        self.available_links.update_internal_data_structures()
        self.selected.update_internal_data_structures()

    def is_valid(self) -> bool:
        if not self.selected.is_valid():
            return False

        return True

    def dump_settings(self) -> List[dict]:
        res = []

        for row in self.selected.rowCount():
            res.append(
                {
                    ControlSufraceField.LINK_NAME.name: self.selected.link_name(row),
                    ControlSufraceField.MIN_ANGLE.name: self.selected.min_angle(row),
                    ControlSufraceField.MAX_ANGLE.name: self.selected.max_angle(row),
                    ControlSufraceField.MAX_ANGLE_RATE.name: self.selected.max_angle_rate(row),
                    ControlSufraceField.LIFT_COEF.name: self.selected.c_lift_delta(row),
                    ControlSufraceField.DRAG_COEF.name: self.selected.c_drag_delta(row),
                    ControlSufraceField.SIDE_COEF.name: self.selected.c_side_delta(row),
                    ControlSufraceField.ROLL_COEF.name: self.selected.c_roll_delta(row),
                    ControlSufraceField.PITCH_COEF.name: self.selected.c_pitch_delta(row),
                    ControlSufraceField.YAW_COEF.name: self.selected.c_yaw_delta(row),
                }
            )

        return res

    def load_settings(self, data: List[dict]) -> None:
        for setting in data:
            link_name = setting[ControlSufraceField.LINK_NAME.name]

            # リンクをAvailableからSelectedに移動させる
            self.available_links.delete_link(link_name)
            self.selected.add_link(
                link_name,
                default_min_angle=data[ControlSufraceField.MIN_ANGLE.name],
                default_max_angle=data[ControlSufraceField.MAX_ANGLE.name],
                default_max_angle_rate=data[ControlSufraceField.MAX_ANGLE_RATE.name],
                default_c_lift_delta=data[ControlSufraceField.LIFT_COEF.name],
                default_c_drag_delta=data[ControlSufraceField.DRAG_COEF.name],
                default_c_side_delta=data[ControlSufraceField.SIDE_COEF.name],
                default_c_roll_delta=data[ControlSufraceField.ROLL_COEF.name],
                default_c_pitch_delta=data[ControlSufraceField.PITCH_COEF.name],
                default_c_yaw_delta=data[ControlSufraceField.ROLL_COEF.name],
            )

    def control_surfaces(self) -> List[ControlSurface]:
        res = [ControlSurface() for _ in range(self.selected.count())]
        for i in range(self.selected.count()):
            res[i].joint_name = self.selected.joint_name(i)
            res[i].min_angle = self.selected.min_angle(i)
            res[i].max_angle = self.selected.max_angle(i)
            res[i].max_angle_rate = self.selected.max_angle_rate(i)
            res[i].c_lift_delta = self.selected.c_lift_delta(i)
            res[i].c_drag_abs_delta = self.selected.c_drag_delta(i)
            res[i].c_side_delta = self.selected.c_side_delta(i)
            res[i].c_roll_delta = self.selected.c_roll_delta(i)
            res[i].c_pitch_delta = self.selected.c_pitch_delta(i)
            res[i].c_yaw_delta = self.selected.c_yaw_delta(i)

        return res


class AvailableLinksWidget(QListWidget):
    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    def update_internal_data_structures(self) -> None:
        """
        以下の条件を満たすリンクの名前の配列を返す．
        - 回転関節 (Revolute) をもつ．
        - リミットが正しく設定されている．
        - Transmissionをもたない．
        - エンドリンクである．
        - 親リンクがルートリンクに固定されている．
        """
        self.clear()

        root_link = self._main.urdf_parser.get_root()
        links = self._main.urdf_parser.get_links()

        for link in links:
            if link.name == root_link.name:
                continue

            joint = self._main.urdf_parser.get_joint(link.name)
            parent = self._main.urdf_parser.get_parent(link.name)

            # 回転関節 (Revolute) をもつ
            if joint.type != JointType.REVOLUTE:
                continue

            # リミットが正しく設定されている
            if not joint.limit.lower < 0.0 < joint.limit.upper:
                continue
            if joint.limit.velocity <= 0.0:
                continue
            if joint.limit.effort <= 0.0:
                continue

            # Transmissionをもたない
            if self._main.urdf_parser.hardware_interface(joint.name) != None:
                continue

            # エンドリンクである
            if not self._main.urdf_parser.is_end_link(link.name):
                continue

            # 親リンクがルートリンクに固定されている
            if not self._main.urdf_parser.is_fixed_link(parent.name):
                continue

            self.add_link(link.name)

        self.sortItems()

    def selected_link(self) -> Union[str, None]:
        try:
            return self.currentItem().text()
        except:
            return None

    def add_link(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(link_name), f"Unknown link: {link_name}"
        assert not self._link_exists_in_list(link_name), f"Duplicated: {link_name}"

        self.addItem(link_name)
        self.sortItems()

    def delete_link(self, link_name: str) -> None:
        items = self.findItems(link_name, Qt.MatchExactly)
        assert len(items) == 1
        self.takeItem(self.row(items[0]))

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0


class SelectedLinksWidget(TableWidget):
    COL_WIDTH = 120
    COEF_DECIMALS = 6
    ANGLE_LIMIT = math.pi / 4

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(0, len(ControlSufraceField))
        self._main = main

        self.setHorizontalHeaderLabels([title_from_snake(item.name) for item in ControlSufraceField])
        for c in range(self.columnCount()):
            self.setColumnWidth(c, self.COL_WIDTH)

    def update_internal_data_structures(self) -> None:
        self.remove_all()

    def is_valid(self) -> bool:
        return True

    def count(self) -> int:
        """選択テーブル内のプロペラの個数を返す．"""
        return self.rowCount()

    def link_name(self, row: int) -> str:
        cell: QLabel = self.cellWidget(row, ControlSufraceField.LINK_NAME.value)
        return cell.text()

    def joint_name(self, row: int) -> str:
        cell: QLabel = self.cellWidget(row, ControlSufraceField.JOINT_NAME.value)
        return cell.text()

    def min_angle(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.MIN_ANGLE.value)
        return cell.value()

    def max_angle(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.MAX_ANGLE.value)
        return cell.value()

    def max_angle_rate(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.MAX_ANGLE_RATE.value)
        return cell.value()

    def c_lift_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.LIFT_COEF.value)
        return cell.value()

    def c_drag_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.DRAG_COEF.value)
        return cell.value()

    def c_side_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.SIDE_COEF.value)
        return cell.value()

    def c_roll_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.ROLL_COEF.value)
        return cell.value()

    def c_pitch_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.PITCH_COEF.value)
        return cell.value()

    def c_yaw_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, ControlSufraceField.YAW_COEF.value)
        return cell.value()

    def get_link_names(self) -> List[str]:
        """選択テーブル内のリンクの名前のリストを返す．"""
        return [self.link_name(row) for row in range(self.count())]

    def get_joint_names(self) -> List[str]:
        """選択テーブル内のジョイントの名前のリストを返す．"""
        return [self.joint_name(row) for row in range(self.count())]

    def selected_link(self) -> Union[str, None]:
        row = self.currentRow()
        return self.link_name(row) if row >= 0 else None

    def add_link(
        self,
        link_name: str,
        default_min_angle: Optional[float] = None,
        default_max_angle: Optional[float] = None,
        default_max_angle_rate: Optional[float] = None,
        default_c_lift_delta: Optional[float] = None,
        default_c_drag_delta: Optional[float] = None,
        default_c_side_delta: Optional[float] = None,
        default_c_roll_delta: Optional[float] = None,
        default_c_pitch_delta: Optional[float] = None,
        default_c_yaw_delta: Optional[float] = None,
    ) -> None:
        joint = self._main.urdf_parser.get_joint(link_name)

        row = self.rowCount()
        self.insertRow(row)

        link_name_label = QLabel(link_name)
        link_name_label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        link_name_label.setAlignment(Qt.AlignCenter)
        self.setCellWidget(row, ControlSufraceField.LINK_NAME.value, link_name_label)

        joint_name_label = QLabel(joint.name)
        joint_name_label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        joint_name_label.setAlignment(Qt.AlignCenter)
        self.setCellWidget(row, ControlSufraceField.JOINT_NAME.value, joint_name_label)

        min_angle = DoubleSpinBox()
        min_angle.setMinimum(-self.ANGLE_LIMIT)
        min_angle.setMaximum(0.0)
        min_angle.setDecimals(3)
        min_angle.setSuffix(" rad")
        min_angle.setValue(default_min_angle if default_min_angle else joint.limit.lower)
        self.setCellWidget(row, ControlSufraceField.MIN_ANGLE.value, min_angle)

        max_angle = DoubleSpinBox()
        max_angle.setMinimum(0.0)
        max_angle.setMaximum(self.ANGLE_LIMIT)
        max_angle.setDecimals(3)
        max_angle.setSuffix(" rad")
        max_angle.setValue(default_max_angle if default_max_angle else joint.limit.upper)
        self.setCellWidget(row, ControlSufraceField.MAX_ANGLE.value, max_angle)

        max_angle_rate = DoubleSpinBox()
        max_angle_rate.setMinimum(1e-3)
        max_angle_rate.setDecimals(3)
        max_angle_rate.setSuffix(" rad/s")
        max_angle_rate.setValue(default_max_angle_rate if default_max_angle_rate else joint.limit.velocity)
        self.setCellWidget(row, ControlSufraceField.MAX_ANGLE_RATE.value, max_angle_rate)

        c_lift_delta = DoubleSpinBox()
        c_lift_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_lift_delta.setSuffix(" /rad")
        c_lift_delta.setValue(default_c_lift_delta if default_c_lift_delta else 0.0)
        self.setCellWidget(row, ControlSufraceField.LIFT_COEF.value, c_lift_delta)

        c_drag_delta = DoubleSpinBox()
        c_drag_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_drag_delta.setSuffix(" /rad")
        c_drag_delta.setValue(default_c_drag_delta if default_c_drag_delta else 0.0)
        self.setCellWidget(row, ControlSufraceField.DRAG_COEF.value, c_drag_delta)

        c_side_delta = DoubleSpinBox()
        c_side_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_side_delta.setSuffix(" /rad")
        c_side_delta.setValue(default_c_side_delta if default_c_side_delta else 0.0)
        self.setCellWidget(row, ControlSufraceField.SIDE_COEF.value, c_side_delta)

        c_roll_delta = DoubleSpinBox()
        c_roll_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_roll_delta.setSuffix(" /rad")
        c_roll_delta.setValue(default_c_roll_delta if default_c_roll_delta else 0.0)
        self.setCellWidget(row, ControlSufraceField.ROLL_COEF.value, c_roll_delta)

        c_pitch_delta = DoubleSpinBox()
        c_pitch_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_pitch_delta.setSuffix(" /rad")
        c_pitch_delta.setValue(default_c_pitch_delta if default_c_pitch_delta else 0.0)
        self.setCellWidget(row, ControlSufraceField.PITCH_COEF.value, c_pitch_delta)

        c_yaw_delta = DoubleSpinBox()
        c_yaw_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_yaw_delta.setSuffix(" /rad")
        c_yaw_delta.setValue(default_c_yaw_delta if default_c_yaw_delta else 0.0)
        self.setCellWidget(row, ControlSufraceField.YAW_COEF.value, c_yaw_delta)

    def delete_link(self, link_name: str) -> None:
        row = self.currentRow()
        assert self.link_name(row) == link_name

        self.removeRow(row)


class AddDeleteButtonsWidget(QWidget):
    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    def __init__(
        self, signals: Signals, available_links: AvailableLinksWidget, selected_links: SelectedLinksWidget
    ) -> None:
        super().__init__()

        self._signals = signals
        self._available_links = available_links
        self._selected_links = selected_links

        cols = QHBoxLayout()
        self.setLayout(cols)

        self._add_button = QPushButton("⬇")
        self._add_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._add_button.clicked.connect(self._on_add_button_clicked)
        cols.addWidget(self._add_button)

        self._delete_button = QPushButton("⬆")
        self._delete_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._delete_button.clicked.connect(self._on_delete_button_clicked)
        cols.addWidget(self._delete_button)

    @pyqtSlot()
    def _on_add_button_clicked(self) -> None:
        selected_link = self._available_links.selected_link()
        if selected_link is None:
            q_error(self, "No link is selected.")
            return

        self._available_links.delete_link(selected_link)
        self._selected_links.add_link(selected_link)

        self._signals.airframe_updated.emit()

    @pyqtSlot()
    def _on_delete_button_clicked(self) -> None:
        selected_link = self._selected_links.selected_link()
        if selected_link is None:
            q_error(self, "No link is selected.")
            return

        self._available_links.add_link(selected_link)
        self._selected_links.delete_link(selected_link)

        self._signals.airframe_updated.emit()
