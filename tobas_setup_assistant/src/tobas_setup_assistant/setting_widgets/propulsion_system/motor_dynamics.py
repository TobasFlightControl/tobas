from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

import numpy as np
from numpy import linalg as LA
from abc import abstractmethod
from overrides import override
from typing import Tuple
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from tobas_tools_py.math import rpm2rps
from tobas_rqt_tools.messages import q_error_named

from ...common import TO_DO, Description
from ...parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_DoubleTable
from .common import PROPULSION_SYSTEM


class MotorDynamicsWidget_Base(QWidget):  # NOTE: ABCを継承するとバグる
    NAME = TO_DO
    ABST_TEXT = TO_DO

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(self.ABST_TEXT)
        self._rows.addWidget(abst)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def rot_speed_coefs(self) -> Tuple[float, float]:
        """V = a w + b w^2 (V[V], w[rad/s])"""
        raise NotImplementedError()

    def copy_from(self, src: MotorDynamicsWidget_Base) -> None:
        for i in range(self._param_rows.count()):
            param_des: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param_src: ParamGetterWidget = src._param_rows.itemAt(i).widget()
            param_des.set(param_src.get())

    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()
        return res

    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])

    def _add_param_widget(self, widget: ParamGetterWidget) -> None:
        """ここで追加したパラメータはcopy, dump, loadが自動で行われる．"""
        self._param_rows.addWidget(widget)


class MotorDynamicsWidget_Spec(MotorDynamicsWidget_Base):
    NAME = "Set from motor spec"
    ABST_TEXT = "Estimate the motor dynamics from the motor's Kv value and internal registance."

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        kv_description = "Motor's rotational speed under no load, relative to the supplied voltage."
        self._kv = ParamGetterWidget_SpinBox(
            "Kv", kv_description, minimum=1, maximum=10**5, default=920, suffix=" rpm/V"
        )
        self._add_param_widget(self._kv)

        resistance_description = "Internal resistance value of the motor."
        self._resistance = ParamGetterWidget_SpinBox(
            "Internal Registance", resistance_description, minimum=1, default=250, suffix=" mΩ"
        )
        self._add_param_widget(self._resistance)

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def rot_speed_coefs(self) -> Tuple[float, float]:
        kv_si = rpm2rps(self._kv.get())  # [rad/s/V]
        R = self._resistance.get() * 1e-3  # [Ω]

        # 発電係数とトルク定数の関係: https://en.wikipedia.org/wiki/Motor_constants
        ke = 1 / kv_si  # 発電係数 [Vs/rad]
        kt = 1 / kv_si  # トルク定数 [Nm/A]

        # 空力特性を取得
        aerodynamics = self._main.propulsion_system.selected.get_aerodynamics(self._link_name)

        a = ke
        b = R * aerodynamics.motor_const() * aerodynamics.moment_const() / kt
        return a, b


class MotorDynamicsWidget_Experiment(MotorDynamicsWidget_Base):
    NAME = "Set from experimental data (recommended)"
    ABST_TEXT = (
        "Estimate the motor dynamics from the data obtained from the Thrust Stand. "
        "Please input the data directly or load it from a CSV file. "
        "Ensure that the experiments are conducted using the battery intended for the aircraft "
        "and with the actual propeller attached."
    )

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        self._data = ParamGetterWidget_DoubleTable("Experimental data", ["Throttle", "Voltage", "RPM"])
        self._data.set_minimum([1.0, 1.0, 1.0])
        self._data.set_maximum([100.0, 1e9, 1e9])
        self._data.set_decimals([0, 6, 0])
        self._data.set_suffix([" %", " V", " rpm"])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._add_param_widget(self._data)

    @override
    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Experiment data is blank.")
            return False

        return True

    @override
    def rot_speed_coefs(self) -> Tuple[float, float]:
        # TODO: 外れ値を除去
        # TODO: あまりにモデルからかけ離れていたら警告を出す

        # データを取得
        throttle, battery_voltage, rpm = np.hsplit(np.array(self._data.get()), 3)
        motor_voltage = battery_voltage * throttle / 100.0
        omega = rpm2rps(rpm)

        # 最小二乗法で係数を推定
        X = np.c_[omega, omega**2]
        a, b = LA.lstsq(X, motor_voltage, rcond=None)[0].squeeze()

        return a, b
