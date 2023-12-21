from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import overrides
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_spacer, DoubleSpinBox, ComboBox
from kdl_sympy.joint import *

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
        abst_text = "推進システム，固定翼舵面以外のTransmissionを持つ関節の設定を行います．"
        super().__init__(main, title_text, abst_text)

        self._available_joints: List[str] = []

        self._table = QTableWidget(0, len(self.LABELS))
        self._table.setHorizontalHeaderLabels(self.LABELS)
        for c in range(self._table.columnCount()):
            self._table.setColumnWidth(c, self.COL_WIDTH)
        self._rows.addWidget(self._table)

        add_spacer(self._rows)

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._main.urdf_parser.robot_model_loaded.connect(self._on_robot_model_loaded)

    @overrides
    def is_valid(self) -> bool:
        return True

    def count(self) -> int:
        return self._table.rowCount()

    def joint_name(self, row: int) -> str:
        jnt_name: QLabel = self._table.cellWidget(row, 0)
        return jnt_name.text()

    def joint_names(self) -> List[str]:
        return [self.joint_name(row) for row in range(self.count())]

    def home_position(self, row: int) -> float:
        home_pos: DoubleSpinBox = self._table.cellWidget(row, 1)
        return home_pos.value()

    def min_position(self, row: int) -> float:
        min_pos: DoubleSpinBox = self._table.cellWidget(row, 2)
        return min_pos.value()

    def max_position(self, row: int) -> float:
        max_pos: DoubleSpinBox = self._table.cellWidget(row, 3)
        return max_pos.value()

    def command_type(self, row: int) -> str:
        cmd_type: ComboBox = self._table.cellWidget(row, 4)
        return cmd_type.currentText()

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

    def p_gain(self, row: int) -> float:
        p_gain: DoubleSpinBox = self._table.cellWidget(row, 5)
        return p_gain.value()

    def i_gain(self, row: int) -> float:
        i_gain: DoubleSpinBox = self._table.cellWidget(row, 6)
        return i_gain.value()

    def d_gain(self, row: int) -> float:
        d_gain: DoubleSpinBox = self._table.cellWidget(row, 7)
        return d_gain.value()

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
                min_pos.setValue(joint.limit.lower)
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
