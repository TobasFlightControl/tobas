from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import math
from typing import List, Union
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import DoubleSpinBox
from dh_rqt_tools.messages import q_error
from kdl_sympy.joint import JointType

from ...parameter_getters import *
from ...constants import *


class ControlSurface:

    def __init__(self) -> None:
        self.min_angle = 0.
        self.max_angle = 0.
        self.max_angle_rate = 0.

        self.c_lift_delta = 0.      # [/rad]
        self.c_drag_abs_delta = 0.  # [/rad]
        self.c_side_delta = 0.      # [/rad]
        self.c_roll_delta = 0.      # [/rad]
        self.c_pitch_delta = 0.     # [/rad]
        self.c_yaw_delta = 0.       # [/rad]


class ControlSurfacesWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.control_surfaces: List[ControlSurface] = []

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel("Control Surfaces")
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(label)

        available_links_label = QLabel("Available Links")
        available_links_label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        available_links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(available_links_label)

        self.available_links = AvailableLinksWidget(main)
        self._rows.addWidget(self.available_links)

        self.add_delete = AddDeleteButtonsWidget(main)
        self._rows.addWidget(self.add_delete)

        self.selected = SelectedLinksWidget(main)
        self._rows.addWidget(self.selected)

    def define_connections(self) -> None:
        self.selected.define_connections()
        self.add_delete.define_connections()
        self.available_links.define_connections()

    def is_valid(self) -> bool:
        if not self.selected.is_valid():
            return False

        return True


class AvailableLinksWidget(QListWidget):

    link_added = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)
        self._main.settings.fixed_wing.control_surfaces.add_delete.delete.connect(
            self._add_selected_link)
        self._main.settings.fixed_wing.control_surfaces.selected.link_added.connect(
            self._delete_selected_link)

    def add_link(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(link_name), f'Unknown link: {link_name}'
        assert not self._link_exists_in_list(link_name), f'Duplicated: {link_name}'
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
        - エンドリンクである．
        - 親リンクがルートリンクに固定されている．
        """
        root_link = self._main.urdf_parser.get_root()
        links = self._main.urdf_parser.get_links()

        for link in links:
            if link.name == root_link.name:
                continue

            joint = self._main.urdf_parser.get_joint(link.name)
            if joint.type != JointType.REVOLUTE:
                continue

            if not self._main.urdf_parser.is_end_link(link.name):
                continue

            parent = self._main.urdf_parser.get_parent(link.name)
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

    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100

    add = pyqtSignal()
    delete = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._add_button = QPushButton("⬇")
        self._add_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self._cols.addWidget(self._add_button)

        self._delete_button = QPushButton("⬆")
        self._delete_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self._cols.addWidget(self._delete_button)

    def define_connections(self) -> None:
        self._add_button.clicked.connect(self._add_button_clicked)
        self._delete_button.clicked.connect(self._delete_button_clicked)

    @pyqtSlot()
    def _add_button_clicked(self) -> None:
        self.add.emit()

    @pyqtSlot()
    def _delete_button_clicked(self) -> None:
        self.delete.emit()


class SelectedLinksWidget(QTableWidget):

    COL_WIDTH = 120
    LABELS = [
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
    ]

    link_added = pyqtSignal(str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(0, len(self.LABELS))
        self._main = main

        self.link_names: List[QLabel] = []
        self.joint_names: List[QLabel] = []
        self.min_angles: List[DoubleSpinBox] = []
        self.max_angles: List[DoubleSpinBox] = []
        self.max_angle_rates: List[DoubleSpinBox] = []
        self.c_lift_delta: List[DoubleSpinBox] = []
        self.c_drag_delta: List[DoubleSpinBox] = []
        self.c_side_delta: List[DoubleSpinBox] = []
        self.c_roll_delta: List[DoubleSpinBox] = []
        self.c_pitch_delta: List[DoubleSpinBox] = []
        self.c_yaw_delta: List[DoubleSpinBox] = []

        self.setHorizontalHeaderLabels(self.LABELS)

        for c in range(self.columnCount()):
            self.setColumnWidth(c, self.COL_WIDTH)

    def define_connections(self) -> None:
        # 必ずAdd -> Deleteの順に実行する
        self._main.settings.fixed_wing.control_surfaces.add_delete.add.connect(
            self._add_selected_link)
        self._main.settings.fixed_wing.control_surfaces.available_links.link_added.connect(
            self._delete_cur_row)

    def is_valid(self) -> bool:
        return True

    def selected_link(self) -> Union[str, None]:
        row = self.currentRow()
        if row < 0:
            return None
        # return self.cellWidget(row, 0).property("text")
        return self.link_names[row].text()

    def count(self) -> int:
        """ 選択テーブル内のプロペラの個数を返す． """
        return len(self.link_names)

    def get_link_names(self) -> List[str]:
        """ 選択テーブル内のリンクの名前のリストを返す． """
        return [link_name.text() for link_name in self.link_names]

    def get_joint_names(self) -> List[str]:
        """ 選択テーブル内のジョイントの名前のリストを返す． """
        return [joint_name.text() for joint_name in self.joint_names]

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.fixed_wing.control_surfaces.available_links.selected_link()
        if selected_link is None:
            q_error(self._main, "No link is selected.")
            return

        row = self.rowCount()
        self.insertRow(row)

        link_name = QLabel(selected_link)
        link_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        link_name.setAlignment(Qt.AlignCenter)
        self.link_names.append(link_name)
        self.setCellWidget(row, 0, link_name)

        joint_name = QLabel(self._main.urdf_parser.get_joint(selected_link).name)
        joint_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        joint_name.setAlignment(Qt.AlignCenter)
        self.joint_names.append(joint_name)
        self.setCellWidget(row, 1, joint_name)

        min_angle = DoubleSpinBox()
        min_angle.setMinimum(-math.pi / 4)
        min_angle.setMaximum(0.)
        min_angle.setDecimals(3)
        min_angle.setSuffix(" rad")
        self.min_angles.append(min_angle)
        self.setCellWidget(row, 2, min_angle)

        max_angle = DoubleSpinBox()
        max_angle.setMinimum(0.)
        max_angle.setMaximum(math.pi / 4)
        max_angle.setDecimals(3)
        max_angle.setSuffix(" rad")
        self.max_angles.append(max_angle)
        self.setCellWidget(row, 3, max_angle)

        max_angle_rate = DoubleSpinBox()
        max_angle_rate.setMinimum(1e-3)
        max_angle_rate.setDecimals(3)
        max_angle_rate.setSuffix(" rad/s")
        self.max_angle_rates.append(max_angle_rate)
        self.setCellWidget(row, 4, max_angle_rate)

        c_lift_delta = DoubleSpinBox()
        c_lift_delta.setDecimals(3)
        c_lift_delta.setSuffix(" /rad")
        self.c_lift_delta.append(c_lift_delta)
        self.setCellWidget(row, 5, c_lift_delta)

        c_drag_delta = DoubleSpinBox()
        c_drag_delta.setDecimals(3)
        c_drag_delta.setSuffix(" /rad")
        self.c_drag_delta.append(c_drag_delta)
        self.setCellWidget(row, 6, c_drag_delta)

        c_side_delta = DoubleSpinBox()
        c_side_delta.setDecimals(3)
        c_side_delta.setSuffix(" /rad")
        self.c_side_delta.append(c_side_delta)
        self.setCellWidget(row, 7, c_side_delta)

        c_roll_delta = DoubleSpinBox()
        c_roll_delta.setDecimals(3)
        c_roll_delta.setSuffix(" /rad")
        self.c_roll_delta.append(c_roll_delta)
        self.setCellWidget(row, 8, c_roll_delta)

        c_pitch_delta = DoubleSpinBox()
        c_pitch_delta.setDecimals(3)
        c_pitch_delta.setSuffix(" /rad")
        self.c_pitch_delta.append(c_pitch_delta)
        self.setCellWidget(row, 9, c_pitch_delta)

        c_yaw_delta = DoubleSpinBox()
        c_yaw_delta.setDecimals(3)
        c_yaw_delta.setSuffix(" /rad")
        self.c_yaw_delta.append(c_yaw_delta)
        self.setCellWidget(row, 10, c_yaw_delta)

        if row == 0:
            # 1段目
            min_angle.setValue(math.radians(-20.))
            max_angle.setValue(math.radians(20.))
            max_angle_rate.setValue(10.)
        else:
            # 2段目以降
            min_angle.setValue(self.min_angles[row - 1].value())
            max_angle.setValue(self.max_angles[row - 1].value())
            max_angle_rate.setValue(self.max_angle_rates[row - 1].value())

        self.link_added.emit(selected_link)

    @pyqtSlot()
    def _delete_cur_row(self) -> None:
        row = self.currentRow()
        if row < 0:
            return

        self.removeRow(row)

        self.link_names.pop(row)
        self.joint_names.pop(row)
        self.min_angles.pop(row)
        self.max_angles.pop(row)
        self.max_angle_rates.pop(row)
        self.c_lift_delta.pop(row)
        self.c_drag_delta.pop(row)
        self.c_side_delta.pop(row)
        self.c_roll_delta.pop(row)
        self.c_pitch_delta.pop(row)
        self.c_yaw_delta.pop(row)
