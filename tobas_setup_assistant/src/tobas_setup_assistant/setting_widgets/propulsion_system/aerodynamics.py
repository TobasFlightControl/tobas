from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from ...parameter_getters import ParamGetterWidget

import math
import numpy as np
from abc import abstractmethod
from overrides import override
from typing import List
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from tobas_tools_py.math import rpm2rps
from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ...common import AIR_DENSITY, TO_DO, Description
from ...parameter_getters import ParamGetterWidget_DoubleSpinBox, ParamGetterWidget_DoubleTable
from .common import PROPULSION_SYSTEM
from .base import BaseSelectedLinkSettingWidget
from .blade_theory import BladeTheory


class AerodynamicsWidget(BaseSelectedLinkSettingWidget):
    NAME = "Aerodynamics"

    NO_SELECT = "Select Setting Method"
    METHOD_NAME_KEY = "method_name"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._methods: List[AerodynamicsWidget_Base] = [
            AerodynamicsWidget_Manual(main, link_name),
            AerodynamicsWidget_BladeTheory(main, link_name),
            AerodynamicsWidget_ThrustStand(main, link_name),
            AerodynamicsWidget_UIUC(main, link_name),
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
            q_error_named(self._main, PROPULSION_SYSTEM, "Please select aerodynamics setting method.")
            return False
        else:
            if not self._selected().is_valid():
                return False

        return True

    @override
    def copy_from(self, src: AerodynamicsWidget) -> None:
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

    def motor_const(self) -> float:
        """[kg*m/rad^2]"""
        return self._selected().motor_const()

    def moment_const(self) -> float:
        """[m]"""
        return self._selected().moment_const()

    def rotor_drag_coef(self) -> float:
        """[kg/rad]"""
        return self._selected().rotor_drag_coef()

    def _selected(self) -> AerodynamicsWidget_Base:
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


class AerodynamicsWidget_Base(QWidget):
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
    def motor_const(self) -> float:
        """[kg*m/rad^2]"""
        raise NotImplementedError()

    @abstractmethod
    def moment_const(self) -> float:
        """[m]"""
        raise NotImplementedError()

    @abstractmethod
    def rotor_drag_coef(self) -> float:
        """[kg/rad]"""
        raise NotImplementedError()

    def copy_from(self, src: AerodynamicsWidget_Base) -> None:
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


class AerodynamicsWidget_Manual(AerodynamicsWidget_Base):
    NAME = "Set Manually"
    ABST_TEXT = "Directly set the propeller aerodynamic constants."

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        motor_const_description = (
            "Propeller thrust constant. "
            "If the thrust constant is denoted as c [kg*m/rad^2] and the rotational speed as w [rad/s], "
            "the thrust force T [N] generated perpendicular to the rotational plane is expressed as T = c w^2."
        )
        self._motor_const = ParamGetterWidget_DoubleSpinBox(
            "Motor Constant",
            motor_const_description,
            decimals=12,
            minimum=0.0,
            default=8.54858e-6,
            suffix=" kg m/rad^2",
        )
        self._add_param_widget(self._motor_const)

        moment_const_description = (
            "Propeller torque reaction coefficient. "
            "If the torque reaction coefficient is c [m] and the propeller's thrust is T [N], "
            "the torque generated in the opposite direction to the propeller's rotation, in Newton-meters, is N = c T."
        )
        self._moment_const = ParamGetterWidget_DoubleSpinBox(
            "Moment Constant", moment_const_description, decimals=6, minimum=0.0, default=0.016, suffix=" m"
        )
        self._add_param_widget(self._moment_const)

        rotor_drag_coef_description = (
            "Propeller drag coefficient. "
            "If the drag coefficient is c [kg/rad], the motor's rotational speed is w [rad/s], "
            "and V [m/s] is the magnitude of the atmospheric velocity component "
            "perpendicular to the rotational axis relative to the aircraft, "
            "then the magnitude of the air drag force F [N] generated on the propeller is expressed as F = c w V."
        )
        self._rotor_drag_coef = ParamGetterWidget_DoubleSpinBox(
            "Rotor Drag Coefficient",
            rotor_drag_coef_description,
            decimals=9,
            minimum=0.0,
            default=8.06428e-5,
            suffix=" kg/rad",
        )
        self._add_param_widget(self._rotor_drag_coef)

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def motor_const(self) -> float:
        return self._motor_const.get()

    @override
    def moment_const(self) -> float:
        return self._moment_const.get()

    @override
    def rotor_drag_coef(self) -> float:
        return self._rotor_drag_coef.get()


class AerodynamicsWidget_BladeTheory(AerodynamicsWidget_Base):
    """Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019]"""

    NAME = "Estimate from Propeller Geometry"
    ABST_TEXT = (
        "Estimate aerodynamic constants using Blade Element Theory or Momentum Theory, "
        "based on the geometric shape of the propeller set above. See "
        "<a href='https://en.wikipedia.org/wiki/Blade_element_theory'>Blade Element Theory</a> and "
        "<a href='https://en.wikipedia.org/wiki/Momentum_theory'>Momentum Theory</a> "
        "for more information."
    )

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def motor_const(self) -> float:
        return self._blade_thory().motor_const()

    @override
    def moment_const(self) -> float:
        return self._blade_thory().moment_const()

    @override
    def rotor_drag_coef(self) -> float:
        return self._blade_thory().rotor_drag_coef()

    def _blade_thory(self) -> BladeTheory:
        propeller = self._main.propulsion_system.selected.get_propeller(self._link_name)
        return BladeTheory(propeller.num_blade(), propeller.radius(), propeller.blade_chord(), propeller.pitch_angle())


class AerodynamicsWidget_ThrustStand(AerodynamicsWidget_Base):
    """
    推力係数とトルク係数はThrust Standの実験データから求める．\\
    空気抗力係数はBlade Theoryから求める．
    """

    NAME = "Estimate from Thrust Stand Data"
    ABST_TEXT = (
        "We estimate the aerodynamic constants from data obtained through Thrust Stand experiments. "
        "For example, see the "
        "<a href='https://www.tytorobotics.com/pages/series-1580-1585'>Tyto Rootics Series 1585 Thrust Stand</a>"
    )  # NOTE: テキスト中に改行コードを入れるとハイパーリンクが機能しない

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        data_description = "Please input experimental data from the Thrust Stand."
        self._data = ParamGetterWidget_DoubleTable(
            "Data from thrust stand", ["RPM", "Thrust", "Torque"], description_text=data_description
        )
        self._data.set_minimum([1e-1, 1e-6, 1e-6])  # TODO: 負の値にも対応
        self._data.set_decimals([0, 6, 6])
        self._data.set_suffix([" rpm", " N", " Nm"])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._add_param_widget(self._data)

    @override
    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Thrust stand data is blank.")
            return False

        return True

    @override
    def motor_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
        rpm, thrust, _ = np.hsplit(np.array(self._data.get()), 3)
        omega2: np.ndarray = rpm2rps(rpm) ** 2
        return ((thrust.T @ omega2) / (omega2.T @ omega2)).item()  # 最小2乗解 (memo: 2-28)

    @override
    def moment_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
        _, thrust, torque = np.hsplit(np.array(self._data.get()), 3)
        return ((torque.T @ thrust) / (thrust.T @ thrust)).item()  # 最小2乗解 (memo: 2-28)

    @override
    def rotor_drag_coef(self) -> float:
        # FIXME: ブレードの幾何形状のみから推定するのではなく，他の空力特性を考慮して推定
        propeller = self._main.propulsion_system.selected.get_propeller(self._link_name)
        blade_theory = BladeTheory(
            propeller.num_blade(), propeller.radius(), propeller.blade_chord(), propeller.pitch_angle()
        )
        return blade_theory.rotor_drag_coef()


class AerodynamicsWidget_UIUC(AerodynamicsWidget_Base):
    NAME = "Estimate from UIUC Propeller Data iSte"
    ABST_TEXT = (
        "If the propeller is listed in the "
        "<a href='https://m-selig.ae.illinois.edu/props/propDB.html'>UIUC Propeller Data Site</a>, "
        "aerodynamic constants measured by research institutions can be utilized."
    )

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        data_description = "Input the Static data for the relevant propeller."
        self._data = ParamGetterWidget_DoubleTable(
            "Measurements in static condition", ["RPM", "CT", "CP"], description_text=data_description
        )
        self._data.set_minimum([1e-3, 1e-6, 1e-6])
        self._data.set_decimals([3, 6, 6])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._add_param_widget(self._data)

    @override
    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Measurements in static condition is blank.")
            return False

        return True

    @override
    def motor_const(self) -> float:
        # CTの平均をとる
        data = np.array(self._data.get())
        CTs = data[:, 1]
        CT = np.mean(CTs)

        propeller = self._main.propulsion_system.selected.get_propeller(self._link_name)
        return (CT * AIR_DENSITY * propeller.diameter() ** 4) / (4 * math.pi ** 2)

    @override
    def moment_const(self) -> float:
        # CT, CPの平均をとる
        # TODO: 単純な平均ではなく，ホバリング時の回転数に対応する値をとる
        data = np.array(self._data.get())
        CTs = data[:, 1]
        CPs = data[:, 2]
        CT = np.mean(CTs)
        CP = np.mean(CPs)

        propeller = self._main.propulsion_system.selected.get_propeller(self._link_name)
        return (propeller.diameter() * CP) / (2 * math.pi * CT)

    @override
    def rotor_drag_coef(self) -> float:
        # FIXME: ブレードの幾何形状のみから推定するのではなく，他の空力特性を考慮して推定
        propeller = self._main.propulsion_system.selected.get_propeller(self._link_name)
        blade_theory = BladeTheory(
            propeller.num_blade(), propeller.radius(), propeller.blade_chord(), propeller.pitch_angle()
        )
        return blade_theory.rotor_drag_coef()
