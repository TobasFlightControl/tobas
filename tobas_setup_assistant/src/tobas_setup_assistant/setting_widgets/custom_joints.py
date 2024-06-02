from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from enum import Enum
from overrides import override
from typing import List
from PyQt5.QtWidgets import QLabel

from tobas_std_tools_py.string import title_from_snake
from tobas_rqt_tools.widgets import DoubleSpinBox, ComboBox, TableWidget
from tobas_rqt_tools.messages import q_error_named
from tobas_kdl_sympy.joint import HardwareInterface, JointType

from .base_setting import BaseSettingWidget


class CustomJointField(Enum):
    JOINT_NAME = 0
    HOME_POSITION = 1
    MIN_POSITION = 2
    MAX_POSITION = 3
    COMMAND_TYPE = 4
    P_GAIN = 5
    I_GAIN = 6
    D_GAIN = 7


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

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Custom Joints"
        abst_text = (
            "Configure the settings for joints with transmissions "
            "other than those in the propulsion system and fixed-wing control surfaces."
        )
        super().__init__(main, title_text, abst_text)

        self._table = TableWidget(0, len(CustomJointField))
        self._table.setHorizontalHeaderLabels([title_from_snake(item.name) for item in CustomJointField])
        for c in range(self._table.columnCount()):
            self._table.setColumnWidth(c, self.COL_WIDTH)
        self._rows.addWidget(self._table)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        self._table.remove_all()
        row = 0

        for joint in self._main.urdf_parser.get_joints():
            # ジョイントタイプが回転または直動でない場合はスキップ
            if joint.type not in {JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC}:
                continue

            # トランスミッションを持たない場合はスキップ
            hi = self._main.urdf_parser.hardware_interface(joint.name)
            if hi is None:
                continue

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

            self._table.setCellWidget(row, CustomJointField.JOINT_NAME.value, jnt_name)
            self._table.setCellWidget(row, CustomJointField.HOME_POSITION.value, home_pos)
            self._table.setCellWidget(row, CustomJointField.MIN_POSITION.value, min_pos)
            self._table.setCellWidget(row, CustomJointField.MAX_POSITION.value, max_pos)
            self._table.setCellWidget(row, CustomJointField.COMMAND_TYPE.value, cmd_type)
            self._table.setCellWidget(row, CustomJointField.P_GAIN.value, p_gain)
            self._table.setCellWidget(row, CustomJointField.I_GAIN.value, i_gain)
            self._table.setCellWidget(row, CustomJointField.D_GAIN.value, d_gain)

            row += 1

    @override
    def is_valid(self) -> bool:
        for i in range(self.count()):
            jnt_name = self.get_joint_name(i)
            home_pos = self.get_home_position(i)
            min_pos = self.get_min_position(i)
            max_pos = self.get_max_position(i)
            if min_pos > max_pos:
                q_error_named(self, self.NAME, f'Position limit of joint "{jnt_name}" is invalid.')
                return False
            if home_pos < min_pos or max_pos < home_pos:
                q_error_named(self, self.NAME, f'Home position of joint "{jnt_name}" is out of limit.')
                return False

        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()

        for row in range(self.count()):
            res[self.get_joint_name(row)] = {
                CustomJointField.HOME_POSITION.name: self.get_home_position(row),
                CustomJointField.MIN_POSITION.name: self.get_min_position(row),
                CustomJointField.MAX_POSITION.name: self.get_max_position(row),
                CustomJointField.COMMAND_TYPE.name: self.get_command_type(row),
                CustomJointField.P_GAIN.name: self.get_p_gain(row),
                CustomJointField.I_GAIN.name: self.get_i_gain(row),
                CustomJointField.D_GAIN.name: self.get_d_gain(row),
            }

        return res

    @override
    def load_settings(self, data: dict) -> None:
        for joint_name, data_ in data.items():
            row = self._get_row(joint_name)
            if row < 0:
                q_error_named(self, self.NAME, f'"{joint_name}" does not exist in the custom joint list.')
                continue

            self.set_home_position(row, data_[CustomJointField.HOME_POSITION.name])
            self.set_min_position(row, data_[CustomJointField.MIN_POSITION.name])
            self.set_max_position(row, data_[CustomJointField.MAX_POSITION.name])
            self.set_command_type(row, data_[CustomJointField.COMMAND_TYPE.name])
            self.set_p_gain(row, data_[CustomJointField.P_GAIN.name])
            self.set_i_gain(row, data_[CustomJointField.I_GAIN.name])
            self.set_d_gain(row, data_[CustomJointField.D_GAIN.name])

    def count(self) -> int:
        """ジョイント数．"""
        return self._table.rowCount()

    def get_joint_name(self, row: int) -> str:
        cell: QLabel = self._table.cellWidget(row, CustomJointField.JOINT_NAME.value)
        return cell.text()

    def set_joint_name(self, row: int, joint_name) -> None:
        cell: QLabel = self._table.cellWidget(row, CustomJointField.JOINT_NAME.value)
        cell.setText(joint_name)

    def get_home_position(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.HOME_POSITION.value)
        return cell.value()

    def set_home_position(self, row: int, value: float) -> None:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.HOME_POSITION.value)
        cell.setValue(value)

    def get_min_position(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.MIN_POSITION.value)
        return cell.value()

    def set_min_position(self, row: int, value: float) -> None:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.MIN_POSITION.value)
        cell.setValue(value)

    def get_max_position(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.MAX_POSITION.value)
        return cell.value()

    def set_max_position(self, row: int, value: float) -> None:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.MAX_POSITION.value)
        cell.setValue(value)

    def get_command_type(self, row: int) -> str:
        cell: ComboBox = self._table.cellWidget(row, CustomJointField.COMMAND_TYPE.value)
        return cell.currentText()

    def set_command_type(self, row: int, command_type: str) -> None:
        cell: ComboBox = self._table.cellWidget(row, CustomJointField.COMMAND_TYPE.value)
        cell.setCurrentText(command_type)

    def get_p_gain(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.P_GAIN.value)
        return cell.value()

    def set_p_gain(self, row: int, value: float) -> None:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.P_GAIN.value)
        cell.setValue(value)

    def get_i_gain(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.I_GAIN.value)
        return cell.value()

    def set_i_gain(self, row: int, value: float) -> None:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.I_GAIN.value)
        cell.setValue(value)

    def get_d_gain(self, row: int) -> float:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.D_GAIN.value)
        return cell.value()

    def set_d_gain(self, row: int, value: float) -> None:
        cell: DoubleSpinBox = self._table.cellWidget(row, CustomJointField.D_GAIN.value)
        cell.setValue(value)

    def get_joint_names(self) -> List[str]:
        return [self.get_joint_name(row) for row in range(self.count())]

    def get_controller_type(self, row: int) -> str:
        jnt_name = self.get_joint_name(row)
        hi = self._main.urdf_parser.hardware_interface(jnt_name)
        if hi == HardwareInterface.POSITION:
            group = "position_controllers"
        elif hi == HardwareInterface.VELOCITY:
            group = "velocity_controllers"
        elif hi == HardwareInterface.EFFORT:
            group = "effort_controllers"
        else:
            raise RuntimeError()

        cmd_type = self.get_command_type(row)
        if cmd_type == self.POSITION:
            controller = "JointPositionController"
        elif cmd_type == self.VELOCITY:
            controller = "JointVelocityController"
        elif cmd_type == self.EFFORT:
            controller = "JointEffortController"
        else:
            raise RuntimeError()

        return f"{group}/{controller}"

    def pid_enabled(self, row: int) -> bool:
        jnt_name = self.get_joint_name(row)
        hi = self._main.urdf_parser.hardware_interface(jnt_name)
        cmd_type: str = self.get_command_type(row)

        if hi == HardwareInterface.POSITION and cmd_type == self.POSITION:
            return False
        if hi == HardwareInterface.VELOCITY and cmd_type == self.VELOCITY:
            return False
        if hi == HardwareInterface.EFFORT and cmd_type == self.EFFORT:
            return False
        return True

    def _get_row(self, joint_name: str) -> int:
        for row in range(self.count()):
            if self.get_joint_name(row) == joint_name:
                return row
        else:
            return -1
