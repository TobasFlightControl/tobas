from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import math
from typing import List, Union
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QPushButton, QListWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.widgets import DoubleSpinBox, TableWidget
from tobas_rqt_tools.messages import q_error
from tobas_kdl_sympy.joint import JointType

from ...common import TITLE_PSIZE, LABEL_PSIZE, BODY_PSIZE
from .common import STABILITY_COEF_DECIMALS


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

        label = QLabel("Control Surfaces")
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        rows.addWidget(label)

        available_links_label = QLabel("Available Links")
        available_links_label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        available_links_label.setAlignment(Qt.AlignLeft)
        rows.addWidget(available_links_label)

        self.available_links = AvailableLinksWidget(main)
        rows.addWidget(self.available_links)

        self.add_delete = AddDeleteButtonsWidget(main)
        rows.addWidget(self.add_delete)

        self.selected = SelectedLinksWidget(main)
        rows.addWidget(self.selected)

    def define_connections(self) -> None:
        self.selected.define_connections()
        self.add_delete.define_connections()
        self.available_links.define_connections()

    def is_valid(self) -> bool:
        if not self.selected.is_valid():
            return False

        return True

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
    link_added = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)
        self._main.settings.fixed_wing.control_surfaces.add_delete.delete.connect(self._add_selected_link)
        self._main.settings.fixed_wing.control_surfaces.selected.link_added.connect(self._delete_selected_link)

    def add_link(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(link_name), f"Unknown link: {link_name}"
        assert not self._link_exists_in_list(link_name), f"Duplicated: {link_name}"
        self.addItem(link_name)

    def delete_link(self, link_name: str) -> None:
        links = self.findItems(link_name, Qt.MatchExactly)
        assert len(links) > 0
        for link in links:  # link: PyQt5.QtWidgets.QListWidgetItem
            mathced_link = self.row(link)  # linkの行番号をint型で取得
            self.takeItem(mathced_link)

    def selected_link(self) -> Union[str, None]:
        try:
            return self.currentItem().text()
        except:
            return None

    @pyqtSlot()
    def _add_available_links(self) -> None:
        """
        以下の条件を満たすリンクの名前の配列を返す．
        - 回転関節 (Revolute) をもつ．
        - リミットが正しく設定されている．
        - Transmissionをもたない．
        - エンドリンクである．
        - 親リンクがルートリンクに固定されている．
        """
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

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.fixed_wing.control_surfaces.selected.selected_link()
        if selected_link is None:
            q_error(self._main, "No link is selected.")
            return

        self.add_link(selected_link)
        self.sortItems()

        self.link_added.emit()

    @pyqtSlot(str)
    def _delete_selected_link(self, link_name: str) -> None:
        self.delete_link(link_name)

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0


class AddDeleteButtonsWidget(QWidget):
    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    add = pyqtSignal()
    delete = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        cols = QHBoxLayout()
        self.setLayout(cols)

        self._add_button = QPushButton("⬇")
        self._add_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._add_button)

        self._delete_button = QPushButton("⬆")
        self._delete_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._delete_button)

    def define_connections(self) -> None:
        self._add_button.clicked.connect(self._add_button_clicked)
        self._delete_button.clicked.connect(self._delete_button_clicked)

    @pyqtSlot()
    def _add_button_clicked(self) -> None:
        self.add.emit()

    @pyqtSlot()
    def _delete_button_clicked(self) -> None:
        self.delete.emit()


class SelectedLinksWidget(TableWidget):
    COL_WIDTH = 120
    COEF_DECIMALS = 6
    ANGLE_LIMIT = math.pi / 4
    LABELS = (
        "Link Name",
        "Joint Name",
        "Min Angle",
        "Max Angle",
        "Max Angle Rate",
        "Lift Coef",
        "Drag Coef",
        "Side Coef",
        "Roll Coef",
        "Pitch Coef",
        "Yaw Coef",
    )

    link_added = pyqtSignal(str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(0, len(self.LABELS))
        self._main = main

        self.setHorizontalHeaderLabels(self.LABELS)
        for c in range(self.columnCount()):
            self.setColumnWidth(c, self.COL_WIDTH)

    def define_connections(self) -> None:
        # 必ずAdd -> Deleteの順に実行する
        control_surfaces = self._main.settings.fixed_wing.control_surfaces
        control_surfaces.add_delete.add.connect(self._add_selected_link)
        control_surfaces.available_links.link_added.connect(self._delete_cur_row)

    def is_valid(self) -> bool:
        return True

    def selected_link(self) -> Union[str, None]:
        row = self.currentRow()
        return self.link_name(row) if row >= 0 else None

    def count(self) -> int:
        """選択テーブル内のプロペラの個数を返す．"""
        return self.rowCount()

    def link_name(self, row: int) -> str:
        cell: QLabel = self.cellWidget(row, 0)
        return cell.text()

    def joint_name(self, row: int) -> str:
        cell: QLabel = self.cellWidget(row, 1)
        return cell.text()

    def min_angle(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 2)
        return cell.value()

    def max_angle(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 3)
        return cell.value()

    def max_angle_rate(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 4)
        return cell.value()

    def c_lift_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 5)
        return cell.value()

    def c_drag_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 6)
        return cell.value()

    def c_side_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 7)
        return cell.value()

    def c_roll_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 8)
        return cell.value()

    def c_pitch_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 9)
        return cell.value()

    def c_yaw_delta(self, row: int) -> float:
        cell: DoubleSpinBox = self.cellWidget(row, 10)
        return cell.value()

    def get_link_names(self) -> List[str]:
        """選択テーブル内のリンクの名前のリストを返す．"""
        return [self.link_name(row) for row in range(self.count())]

    def get_joint_names(self) -> List[str]:
        """選択テーブル内のジョイントの名前のリストを返す．"""
        return [self.joint_name(row) for row in range(self.count())]

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.fixed_wing.control_surfaces.available_links.selected_link()
        if selected_link is None:
            q_error(self._main, "No link is selected.")
            return
        joint = self._main.urdf_parser.get_joint(selected_link)

        row = self.rowCount()
        self.insertRow(row)

        link_name = QLabel(selected_link)
        link_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        link_name.setAlignment(Qt.AlignCenter)
        self.setCellWidget(row, 0, link_name)

        joint_name = QLabel(joint.name)
        joint_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        joint_name.setAlignment(Qt.AlignCenter)
        self.setCellWidget(row, 1, joint_name)

        min_angle = DoubleSpinBox()
        min_angle.setMinimum(-self.ANGLE_LIMIT)
        min_angle.setMaximum(0.0)
        min_angle.setDecimals(3)
        min_angle.setSuffix(" rad")
        min_angle.setValue(joint.limit.lower)
        self.setCellWidget(row, 2, min_angle)

        max_angle = DoubleSpinBox()
        max_angle.setMinimum(0.0)
        max_angle.setMaximum(self.ANGLE_LIMIT)
        max_angle.setDecimals(3)
        max_angle.setSuffix(" rad")
        max_angle.setValue(joint.limit.upper)
        self.setCellWidget(row, 3, max_angle)

        max_angle_rate = DoubleSpinBox()
        max_angle_rate.setMinimum(1e-3)
        max_angle_rate.setDecimals(3)
        max_angle_rate.setSuffix(" rad/s")
        max_angle_rate.setValue(joint.limit.velocity)
        self.setCellWidget(row, 4, max_angle_rate)

        c_lift_delta = DoubleSpinBox()
        c_lift_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_lift_delta.setSuffix(" /rad")
        self.setCellWidget(row, 5, c_lift_delta)

        c_drag_delta = DoubleSpinBox()
        c_drag_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_drag_delta.setSuffix(" /rad")
        self.setCellWidget(row, 6, c_drag_delta)

        c_side_delta = DoubleSpinBox()
        c_side_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_side_delta.setSuffix(" /rad")
        self.setCellWidget(row, 7, c_side_delta)

        c_roll_delta = DoubleSpinBox()
        c_roll_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_roll_delta.setSuffix(" /rad")
        self.setCellWidget(row, 8, c_roll_delta)

        c_pitch_delta = DoubleSpinBox()
        c_pitch_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_pitch_delta.setSuffix(" /rad")
        self.setCellWidget(row, 9, c_pitch_delta)

        c_yaw_delta = DoubleSpinBox()
        c_yaw_delta.setDecimals(STABILITY_COEF_DECIMALS)
        c_yaw_delta.setSuffix(" /rad")
        self.setCellWidget(row, 10, c_yaw_delta)

        self.link_added.emit(selected_link)
        self._main.signals.airframe_updated.emit()

    @pyqtSlot()
    def _delete_cur_row(self) -> None:
        row = self.currentRow()
        if row < 0:
            return

        self.removeRow(row)
        self._main.signals.airframe_updated.emit()
