from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import numpy as np
from numpy import linalg as LA
from abc import abstractmethod
from typing import final, Tuple, List
from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.math import rpm2rps
from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from .common import PROPULSION_SYSTEM
from .max_rot_speed import MaxRotationSpeedWidget


class MotorWidget(QWidget):
    NO_SELECT = "Select setting method"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        rows = QVBoxLayout()
        self.setLayout(rows)

        title = QLabel("Motor Settings")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        rows.addWidget(title)

        self._methods: List[MotorWidget_Base] = [
            MotorWidget_MotorSpec(main, link_name),
            MotorWidget_Experiment(main, link_name),
        ]

        self._method_name = ComboBox()
        self._method_name.addItem(self.NO_SELECT)
        rows.addWidget(self._method_name)

        for method in self._methods:
            self._method_name.addItem(method.NAME)
            rows.addWidget(method)

        self._update_visibility()
        self._define_connections()

    def is_valid(self) -> bool:
        if self._method_name.currentText() == self.NO_SELECT:
            print(self._method_name.currentText())
            q_error_named(self._main, PROPULSION_SYSTEM, "Please select motor setting method.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    def max_rot_speed(self) -> float:
        """[rad/s]"""
        return self._selected().max_rot_speed()

    def direction(self) -> str:
        """CW or CCW"""
        return self._selected().direction()

    def num_poles(self) -> int:
        return self._selected().num_poles()

    def time_const_up(self) -> float:
        """[s]"""
        return self._selected().time_const_up()

    def time_const_down(self) -> float:
        """[s]"""
        return self._selected().time_const_down()

    def rot_speed_coefs(self) -> Tuple[float, float]:
        """V = a w + b w^2 (V[V], w[rad/s])"""
        return self._selected().rot_speed_coefs()

    def copy_from(self, src: MotorWidget) -> None:
        self._method_name.setCurrentText(src._method_name.currentText())

        for des_method, src_method in zip(self._methods, src._methods):
            des_method.copy_from(src_method)

        self._update_visibility()

    def _define_connections(self) -> None:
        self._method_name.currentTextChanged.connect(self._on_type_changed)

    def _selected(self) -> MotorWidget_Base:
        method_name = self._method_name.currentText()

        if method_name == self.NO_SELECT:
            raise RuntimeError("Setting method is not selected.")

        for method in self._methods:
            if method_name == method.NAME:
                return method

        raise RuntimeError(f"Invalid setting method: {method_name}")

    def _update_visibility(self) -> None:
        method_name = self._method_name.currentText()

        for method in self._methods:
            method.setVisible(False)

        for method in self._methods:
            if method.NAME == method_name:
                method.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, _: str) -> None:
        self._update_visibility()


class MotorWidget_Base(QWidget):  # NOTE: ABCを継承するとバグる
    NAME = UNKNOWN

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self._max_rot_speed = MaxRotationSpeedWidget(main, link_name)
        self._rows.addWidget(self._max_rot_speed)

        direction_description = (
            "Motor rotation direction. "
            "Please choose either CW (Clockwise) or CCW (Counter Clockwise) relative to the rotation axis. "
            "For instance, in rotary-wing aircraft, "
            "propellers positioned diagonally opposite each other typically rotate in the same direction."
        )
        self._direction = ParamGetterWidget_ComboBox(
            "Rotating Direction",
            direction_description,
            [CW, CCW],
        )
        self._rows.addWidget(self._direction)

        num_poles_description = ""  # TODO
        self._num_poles = ParamGetterWidget_SpinBox(
            "The number of poles",
            num_poles_description,
            minimum=2,
            default=14,
        )
        self._rows.addWidget(self._num_poles)

        time_const_up_description = (
            "Time constant of the motor's response when increasing its rotational speed, "
            "relative to the command value."
        )
        self._time_const_up = ParamGetterWidget_SpinBox(
            "Time Constant Up",
            time_const_up_description,
            minimum=1,
            default=15,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_up)

        time_const_down_description = (
            "Time constant of the motor's response when decreasing its rotational speed, "
            "relative to the command value."
        )
        self._time_const_down = ParamGetterWidget_SpinBox(
            "Time Constant Down",
            time_const_down_description,
            minimum=1,
            default=30,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_down)

    @abstractmethod
    def is_valid(self) -> bool:
        if not self._max_rot_speed.is_valid():
            return False

        if self.num_poles() % 2 == 1:
            q_error_named(self._main, PROPULSION_SYSTEM, "The number of poles of a brushless motor must be even.")
            return False

        return True

    @abstractmethod
    def copy_from(self, src: MotorWidget_Base) -> None:
        self._max_rot_speed.copy_from(src._max_rot_speed)
        self._direction.set(src._direction.get())
        self._num_poles.set(src._num_poles.get())
        self._time_const_up.set(src._time_const_up.get())
        self._time_const_down.set(src._time_const_down.get())

    @abstractmethod
    def rot_speed_coefs(self) -> Tuple[float, float]:
        """V = a w + b w^2 (V[V], w[rad/s])"""
        raise NotImplementedError()

    @final
    def max_rot_speed(self) -> float:
        """[rad/s]"""
        return self._max_rot_speed.max_rot_speed()

    @final
    def direction(self) -> str:
        """CW or CCW"""
        return self._direction.get()

    @final
    def num_poles(self) -> float:
        return self._num_poles.get()

    @final
    def time_const_up(self) -> float:
        """[s]"""
        return self._time_const_up.get() * 1e-3

    @final
    def time_const_down(self) -> float:
        """[s]"""
        return self._time_const_down.get() * 1e-3


class MotorWidget_MotorSpec(MotorWidget_Base):
    NAME = "Set from motor spec"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        kv_description = "Motor's rotational speed under no load, relative to the supplied voltage."
        self._kv = ParamGetterWidget_SpinBox(
            "Kv",
            kv_description,
            minimum=1,
            maximum=10**5,
            default=920,
            suffix=" rpm/V",
        )
        self._rows.addWidget(self._kv)

        resistance_description = "Internal resistance value of the motor."
        self._resistance = ParamGetterWidget_SpinBox(
            "Internal Registance",
            resistance_description,
            minimum=1,
            default=250,
            suffix=" mΩ",
        )
        self._rows.addWidget(self._resistance)

    @overrides
    def is_valid(self) -> bool:
        if not super().is_valid():
            return False

        return True

    @overrides
    def rot_speed_coefs(self) -> Tuple[float, float]:
        kv_si = rpm2rps(self._kv.get())  # [rad/s/V]
        R = self._resistance.get() * 1e-3  # [Ω]

        # 発電係数とトルク定数の関係: https://en.wikipedia.org/wiki/Motor_constants
        ke = 1 / kv_si  # 発電係数 [Vs/rad]
        kt = 1 / kv_si  # トルク定数 [Nm/A]

        # 空力特性を取得
        aero = self._main.settings.propulsion_system.selected.get_aerodynamics(self._link_name)

        a = ke
        b = R * aero.motor_const() * aero.moment_const() / kt
        return a, b

    @overrides
    def copy_from(self, src: MotorWidget_MotorSpec) -> None:
        super().copy_from(src)
        self._kv.set(src._kv.get())


class MotorWidget_Experiment(MotorWidget_Base):
    NAME = "Set from experimental data (recommended)"

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        data_description = (
            "Estimate the relationship between the PWM signal to the ESC "
            "and the motor's rotational speed from Thrust Stand experiment data. "
            "Please input the data directly or load it from a CSV file. "
            "Ensure that the experiments are conducted using the battery intended for the aircraft "
            "and with the actual propeller attached."
        )
        self._data = ParamGetterWidget_DoubleTable(
            "Experimental data",
            ["Throttle", "Voltage", "RPM"],
            description_text=data_description,
        )
        self._data.set_minimum([1.0, 1.0, 1.0])
        self._data.set_maximum([100.0, 1e9, 1e9])
        self._data.set_decimals([0, 6, 0])
        self._data.set_suffix([" %", " V", " rpm"])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._rows.addWidget(self._data)

    @overrides
    def is_valid(self) -> bool:
        if not super().is_valid():
            return False

        if self._data.count() == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Experiment data is blank.")
            return False

        return True

    @overrides
    def rot_speed_coefs(self) -> Tuple[float, float]:
        # TODO: 外れ値を除去
        # TODO: あまりにモデルからかけ離れていたら警告を出す

        # データを取得
        data = self._data.get()
        throttle, battery_voltage, rpm = np.hsplit(data, 3)
        motor_voltage = battery_voltage * throttle / 100.0
        omega = rpm2rps(rpm)

        # 最小二乗法で係数を推定
        X = np.c_[omega, omega**2]
        a, b = LA.lstsq(X, motor_voltage, rcond=None)[0].squeeze()

        return a, b

    @overrides
    def copy_from(self, src: MotorWidget_Experiment) -> None:
        super().copy_from(src)
        self._data.set(src._data.get())
