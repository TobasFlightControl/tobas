from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import overrides
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import DoubleSpinBox, ComboBox
from tobas_rqt_tools.messages import q_error_named
from tobas_kdl_sympy.joint import *

from ..parameter_getters import *
from ..common import *
from .base_setting import BaseSettingWidget


class CustomJointsWidget(BaseSettingWidget):
    NAME = "Custom Joints"

    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"

    DEFAULT_P_GAIN = 10.0
    DEFAULT_I_GAIN = 0.1
    DEFAULT_D_GAIN = 1.0

    COL_WIDTH = 120
    POS_DECIMALS = 3
    GAIN_DECIMALS = 3
    LABELS = (
        "Joint Name",
        "Home Position",
        "Min Position",
        "Max Position",
        "Controller Type",
        "P Gain",
        "I Gain",
        "D Gain",
    )

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Custom Joints"
        abst_text = (
            "Configure the settings for joints with transmissions "
            "other than those in the propulsion system and fixed-wing control surfaces."
        )
        super().__init__(main, title_text, abst_text)

        self._available_joints: List[str] = []

        self._table = QTableWidget(0, len(self.LABELS))
        self._table.setHorizontalHeaderLabels(self.LABELS)
        for c in range(self._table.columnCount()):
            self._table.setColumnWidth(c, self.COL_WIDTH)
        self._rows.addWidget(self._table)

        self._rows.addStretch()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._main.urdf_parser.robot_model_loaded.connect(self._on_robot_model_loaded)

    @overrides
    def is_valid(self) -> bool:
        for i in range(self.count()):
            jnt_name = self.joint_name(i)
            home_pos = self.home_position(i)
            min_pos = self.min_position(i)
            max_pos = self.max_position(i)
            if min_pos > max_pos:
                q_error_named(self, self.NAME, f"Position limit of joint '{jnt_name} is invalid.")
                return False
            if home_pos < min_pos or max_pos < home_pos:
                q_error_named(
                    self,
                    self.NAME,
                    f"Home position of joint '{jnt_name} is out of limit.",
                )
                return False

        return True

    def count(self) -> int:
        return self._table.rowCount()

    def joint_name(self, row: int) -> str:
        cell: QLabel = self._table.cellWidget(row, 0)
        return cell.text()

    def home_position(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, 1)
        return cell.value()

    def min_position(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, 2)
        return cell.value()

    def max_position(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, 3)
        return cell.value()

    def command_type(self, row: int) -> str:
        cell: ComboBox = self._table.cellWidget(row, 4)
        return cell.currentText()

    def p_gain(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, 5)
        return cell.value()

    def i_gain(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, 6)
        return cell.value()

    def d_gain(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, 7)
        return cell.value()

    def pid_enabled(self, row: int) -> bool:
        jnt_name = self.joint_name(row)
        hi = self._main.urdf_parser.hardware_interface(jnt_name)
        cmd_type: str = self.command_type(row)

        if hi == HardwareInterface.POSITION and cmd_type == self.POSITION:
            return False
        if hi == HardwareInterface.VELOCITY and cmd_type == self.VELOCITY:
            return False
        if hi == HardwareInterface.EFFORT and cmd_type == self.EFFORT:
            return False
        return True

    def joint_names(self) -> List[str]:
        return [self.joint_name(row) for row in range(self.count())]

    def controller_type(self, row: int) -> str:
        jnt_name = self.joint_name(row)
        hi = self._main.urdf_parser.hardware_interface(jnt_name)
        if hi == HardwareInterface.POSITION:
            group = "position_controllers"
        elif hi == HardwareInterface.VELOCITY:
            group = "velocity_controllers"
        elif hi == HardwareInterface.EFFORT:
            group = "effort_controllers"
        else:
            raise RuntimeError()

        cmd_type: str = self.command_type(row)
        if cmd_type == self.POSITION:
            controller = "JointPositionController"
        elif cmd_type == self.VELOCITY:
            controller = "JointVelocityController"
        elif cmd_type == self.EFFORT:
            controller = "JointEffortController"
        else:
            raise RuntimeError()

        return f"{group}/{controller}"

    @pyqtSlot()
    def _on_robot_model_loaded(self) -> None:
        row = 0

        for joint in self._main.urdf_parser.get_joints():
            # ジョイントタイプが回転または直動でない場合はスキップ
            if joint.type not in {
                JointType.REVOLUTE,
                JointType.CONTINUOUS,
                JointType.PRISMATIC,
            }:
                continue

            # トランスミッションを持たない場合はスキップ
            hi = self._main.urdf_parser.hardware_interface(joint.name)
            if hi is None:
                continue

            self._available_joints.append(joint.name)
            self._table.insertRow(row)

            jnt_name = QLabel(joint.name)

            home_pos = DoubleSpinBox()
            min_pos = DoubleSpinBox()
            max_pos = DoubleSpinBox()

            home_pos.setDecimals(self.POS_DECIMALS)
            min_pos.setDecimals(self.POS_DECIMALS)
            max_pos.setDecimals(self.POS_DECIMALS)

            home_pos.setValue(0.0)
            if joint.limit is not None:
                home_pos.setMinimum(joint.limit.lower)
                home_pos.setMaximum(joint.limit.upper)
                min_pos.setMinimum(joint.limit.lower)
                min_pos.setMaximum(joint.limit.upper)
                min_pos.setValue(joint.limit.lower)
                max_pos.setMinimum(joint.limit.lower)
                max_pos.setMaximum(joint.limit.upper)
                max_pos.setValue(joint.limit.upper)

            if joint.type == JointType.REVOLUTE or joint.type == JointType.CONTINUOUS:
                home_pos.setSuffix(" rad")
                min_pos.setSuffix(" rad")
                max_pos.setSuffix(" rad")
            elif joint.type == JointType.PRISMATIC:
                home_pos.setSuffix(" m")
                min_pos.setSuffix(" m")
                max_pos.setSuffix(" m")
            else:
                raise RuntimeError(f"Joint type '{joint.type}' is unexpected.")

            cmd_type = ComboBox()
            p_gain = DoubleSpinBox()
            i_gain = DoubleSpinBox()
            d_gain = DoubleSpinBox()

            p_gain.setDecimals(self.GAIN_DECIMALS)
            i_gain.setDecimals(self.GAIN_DECIMALS)
            d_gain.setDecimals(self.GAIN_DECIMALS)

            p_gain.setValue(self.DEFAULT_P_GAIN)
            i_gain.setValue(self.DEFAULT_I_GAIN)
            d_gain.setValue(self.DEFAULT_D_GAIN)

            if hi == HardwareInterface.POSITION:
                cmd_type.addItem(self.POSITION)
                p_gain.setEnabled(False)
                i_gain.setEnabled(False)
                d_gain.setEnabled(False)
            elif hi == HardwareInterface.VELOCITY:
                cmd_type.addItem(self.POSITION)
                cmd_type.addItem(self.VELOCITY)
                cmd_type.setCurrentText(self.VELOCITY)
            elif hi == HardwareInterface.EFFORT:
                cmd_type.addItem(self.POSITION)
                cmd_type.addItem(self.VELOCITY)
                cmd_type.addItem(self.EFFORT)
                cmd_type.setCurrentText(self.EFFORT)
            else:
                raise RuntimeError(f"Unknown hardware interface: {hi.value}")

            self._table.setCellWidget(row, 0, jnt_name)
            self._table.setCellWidget(row, 1, home_pos)
            self._table.setCellWidget(row, 2, min_pos)
            self._table.setCellWidget(row, 3, max_pos)
            self._table.setCellWidget(row, 4, cmd_type)
            self._table.setCellWidget(row, 5, p_gain)
            self._table.setCellWidget(row, 6, i_gain)
            self._table.setCellWidget(row, 7, d_gain)

            row += 1
