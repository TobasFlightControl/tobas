from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from typing import final, List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import DoubleSpinBox
from tobas_rqt_tools.messages import q_error_named

from ...common import *
from ...utils import rps_from_rpm
from .common import PROPULSION_SYSTEM


class MaxRotationSpeedWidget(QWidget):
    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Max Rotation Speed")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        rows.addWidget(label)

        description = Description("")  # TODO
        rows.addWidget(description)

        self._ckb_broup = QButtonGroup()
        self._ckb_broup.setExclusive(True)

        self._methods: List[MaxRotationSpeedMethod] = [
            MaxRotationSpeedMethod_Manual(main, link_name, self._ckb_broup),
            MaxRotationSpeedMethod_Voltage(main, link_name, self._ckb_broup),
            MaxRotationSpeedMethod_Current(main, link_name, self._ckb_broup),
        ]

        for method in self._methods:
            rows.addWidget(method)

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        return self._selected().max_rot_speed()

    def is_valid(self) -> bool:
        for method in self._methods:
            if method.is_checked():
                if not method.is_valid():
                    return False
                break
        else:
            q_error_named(
                self._main,
                PROPULSION_SYSTEM,
                "Please set max rotation speed.",
            )
            return False

        return True

    def copy_from(self, src: MaxRotationSpeedWidget) -> None:
        for des_method, src_method in zip(self._methods, src._methods):
            des_method._checkbox.setChecked(src_method._checkbox.isChecked())
            des_method.copy_from(src_method)

    def _selected(self) -> MaxRotationSpeedMethod:
        for method in self._methods:
            if method.is_checked():
                return method
        else:
            raise RuntimeError("No method is selected.")


class MaxRotationSpeedMethod(QWidget):
    NAME = UNKNOWN

    def __init__(
        self, main: SetupAssistant, link_name: str, ckb_group: QButtonGroup
    ) -> None:
        super().__init__()
        self._main = main
        self._link_name = link_name

        cols = QHBoxLayout()
        self.setLayout(cols)

        self._checkbox = QCheckBox(self.NAME)
        self._checkbox.toggled.connect(self._on_checkbox_toggled)
        cols.addWidget(self._checkbox)
        ckb_group.addButton(self._checkbox)

        self._spinbox = DoubleSpinBox()
        self._spinbox.setEnabled(False)
        cols.addWidget(self._spinbox)

    @abstractmethod
    def max_rot_speed(self) -> float:
        """[rad/s]"""
        raise NotImplementedError()

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @final
    def copy_from(self, src: MaxRotationSpeedMethod) -> None:
        self._spinbox.setValue(src._spinbox.value())

    @final
    def is_checked(self) -> bool:
        return self._checkbox.isChecked()

    @pyqtSlot(bool)
    def _on_checkbox_toggled(self, toggled: bool) -> None:
        if toggled:
            self._spinbox.setEnabled(True)
        else:
            self._spinbox.setEnabled(False)


class MaxRotationSpeedMethod_Manual(MaxRotationSpeedMethod):
    NAME = "Set manually"

    def __init__(
        self, main: SetupAssistant, link_name: str, ckb_group: QButtonGroup
    ) -> None:
        super().__init__(main, link_name, ckb_group)

        self._spinbox.setSuffix(" rpm")
        self._spinbox.setDecimals(0)
        self._spinbox.setMinimum(0)
        self._spinbox.setMaximum(1e5)
        self._spinbox.setValue(1e4)  # NOTE: 最大最小を設定した時に値が変化するため，デフォルト値の設定はその後

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        return rps_from_rpm(self._spinbox.value())

    def is_valid(self) -> bool:
        return True


class MaxRotationSpeedMethod_Voltage(MaxRotationSpeedMethod):
    NAME = "Estimate from maximum input voltage"

    def __init__(
        self, main: SetupAssistant, link_name: str, ckb_group: QButtonGroup
    ) -> None:
        super().__init__(main, link_name, ckb_group)

        self._spinbox.setSuffix(" V")
        self._spinbox.setDecimals(2)
        self._spinbox.setMinimum(0.0)
        self._spinbox.setValue(16.8)

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        motor = self._main.settings.propulsion_system.selected.get_motor(
            self._link_name
        )
        a, b = motor.rot_speed_coefs()
        V = self._spinbox.value()
        return (math.sqrt(a**2 + 4 * b * V) - a) / (2 * b)

    def is_valid(self) -> bool:
        return True


class MaxRotationSpeedMethod_Current(MaxRotationSpeedMethod):
    NAME = "Estimate from maximum continuous current"

    def __init__(
        self, main: SetupAssistant, link_name: str, ckb_group: QButtonGroup
    ) -> None:
        super().__init__(main, link_name, ckb_group)

        self._spinbox.setSuffix(" A")
        self._spinbox.setDecimals(2)
        self._spinbox.setMinimum(0.0)
        self._spinbox.setValue(10.0)

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        selected = self._main.settings.propulsion_system.selected
        motor = selected.get_motor(self._link_name)
        aero = selected.get_aerodynamics(self._link_name)

        kt, _ = motor.rot_speed_coefs()
        motor_const = aero.motor_const()
        moment_const = aero.moment_const()

        max_current = self._spinbox.value()
        max_torque = kt * max_current
        max_thrust = max_torque / moment_const
        max_rot_speed = math.sqrt(max_thrust / motor_const)

        return max_rot_speed

    def is_valid(self) -> bool:
        return True
