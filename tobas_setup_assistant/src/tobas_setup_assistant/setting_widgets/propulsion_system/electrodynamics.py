from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

import numpy as np
from numpy import linalg as LA
from abc import abstractmethod
from overrides import override
from typing import Tuple, List
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from tobas_std_tools_py.math import rpm2rps
from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ...common import TO_DO, Description
from ...parameter_getters import ParamGetterWidget_DoubleTable
from .common import PROPULSION_SYSTEM
from .base import BaseSelectedLinkSettingWidget


class ElectrodynamicsWidget(BaseSelectedLinkSettingWidget):
    NAME = "Electrodynamics"

    NO_SELECT = "Select Setting Method"
    METHOD_NAME_KEY = "method_name"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._methods: List[MotorDynamicsWidget_Base] = [
            MotorDynamicsWidget_Spec(main, link_name),
            MotorDynamicsWidget_Experiment(main, link_name),
        ]

        self._method_name = ComboBox()
        self._method_name.currentTextChanged.connect(self._on_type_changed)
        rows.addWidget(self._method_name)

        self._method_name.addItem(self.NO_SELECT)
        for method in self._methods:
            self._method_name.addItem(method.NAME)
            rows.addWidget(method)

        rows.addStretch()

        self._update_visibility()

    @override
    def is_valid(self) -> bool:
        if self._method_name.currentText() == self.NO_SELECT:
            q_error_named(self._main, PROPULSION_SYSTEM, "Please select motor setting method.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    @override
    def copy_from(self, src: ElectrodynamicsWidget) -> None:
        self._method_name.setCurrentText(src._method_name.currentText())
        for des_method, src_method in zip(self._methods, src._methods):
            des_method.copy_from(src_method)

    @override
    def dump_settings(self) -> dict:
        res = dict()

        res[self.METHOD_NAME_KEY] = self._method_name.currentText()
        for method in self._methods:
            res[method.NAME] = method.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._method_name.setCurrentText(data[self.METHOD_NAME_KEY])
        for method in self._methods:
            method.load_settings(data[method.NAME])

    def rot_speed_coefs(self) -> Tuple[float, float]:
        """V = a w + b w^2 (V[V], w[rad/s])"""
        return self._selected().rot_speed_coefs()

    def _selected(self) -> MotorDynamicsWidget_Base:
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
    NAME = "Estimate from Motor Spec"
    ABST_TEXT = "Estimate the motor dynamics from the motor's Kv value and internal registance."

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def rot_speed_coefs(self) -> Tuple[float, float]:
        motor = self._main.propulsion_system.selected.get_motor(self._link_name)

        kv_si = motor.kv()
        registance = motor.internal_resistance()

        # 発電係数とトルク定数の関係: https://en.wikipedia.org/wiki/Motor_constants
        ke = 1 / kv_si  # 発電係数 [Vs/rad]
        kt = 1 / kv_si  # トルク定数 [Nm/A]

        # 空力特性を取得
        aerodynamics = self._main.propulsion_system.selected.get_aerodynamics(self._link_name)

        a = ke
        b = registance * aerodynamics.motor_const() * aerodynamics.moment_const() / kt
        return a, b


class MotorDynamicsWidget_Experiment(MotorDynamicsWidget_Base):
    NAME = "Estimate from Experimental Data (Recommended)"
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
