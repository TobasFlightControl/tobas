from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import math
import numpy as np
from abc import abstractmethod
from overrides import overrides
from typing import List, final
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.math import rpm2rps
from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from .common import PROPULSION_SYSTEM
from .blade_theory import BladeTheory


class AerodynamicsWidget(QWidget):
    NO_SELECT = "Select setting method"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        rows = QVBoxLayout()
        self.setLayout(rows)

        title = QLabel("Aerodynamics")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        rows.addWidget(title)

        self._methods: List[AerodynamicsWidget_Base] = [
            AerodynamicsWidget_Manual(main, link_name),
            AerodynamicsWidget_BladeTheory(main, link_name),
            AerodynamicsWidget_ThrustStand(main, link_name),
            AerodynamicsWidget_UIUC(main, link_name),
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
            q_error_named(self._main, PROPULSION_SYSTEM, "Please select aerodynamics setting method.")
            return False
        else:
            if not self._selected().is_valid():
                return False

        return True

    def motor_const(self) -> float:
        """[kg*m/rad^2]"""
        return self._selected().motor_const()

    def moment_const(self) -> float:
        """[m]"""
        return self._selected().moment_const()

    def rotor_drag_coef(self) -> float:
        """[kg/rad]"""
        return self._selected().rotor_drag_coef()

    def max_model_error_rate(self) -> float:
        """[-]"""
        return self._selected().max_model_error_rate()

    def copy_from(self, src: AerodynamicsWidget) -> None:
        self._method_name.setCurrentText(src._method_name.currentText())

        for des_method, src_method in zip(self._methods, src._methods):
            des_method.copy_from(src_method)

        self._update_visibility()

    def _define_connections(self) -> None:
        self._method_name.currentTextChanged.connect(self._on_type_changed)

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
    NAME = UNKNOWN

    def __init__(self, main: SetupAssistant, link_name: str, abst_text: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = Description(abst_text)
        self._rows.addWidget(abst)

        max_model_error_rate_description = "Maximum error rate in the modeling of aerodynamic constants."
        self._max_model_error_rate = ParamGetterWidget_SpinBox(
            "Max Model Error Rate",
            max_model_error_rate_description,
            minimum=0,
            maximum=1000,
            default=10,
            suffix=" %",
        )
        self._rows.addWidget(self._max_model_error_rate)

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

    @abstractmethod
    def copy_from(self, src: AerodynamicsWidget_Base) -> None:
        self._max_model_error_rate.set(src._max_model_error_rate.get())

    @final
    def max_model_error_rate(self) -> float:
        """[-]"""
        return self._max_model_error_rate.get() / 100.0


class AerodynamicsWidget_Manual(AerodynamicsWidget_Base):
    NAME = "Set manually"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        abst_text = "Directly set the propeller aerodynamic constants."
        super().__init__(main, link_name, abst_text)

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
        self._rows.addWidget(self._motor_const)

        moment_const_description = (
            "Propeller torque reaction coefficient. "
            "If the torque reaction coefficient is c [m] and the propeller's thrust is T [N], "
            "the torque generated in the opposite direction to the propeller's rotation, in Newton-meters, is N = c T."
        )
        self._moment_const = ParamGetterWidget_DoubleSpinBox(
            "Moment Constant",
            moment_const_description,
            decimals=6,
            minimum=0.0,
            default=0.016,
            suffix=" m",
        )
        self._rows.addWidget(self._moment_const)

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
        self._rows.addWidget(self._rotor_drag_coef)

    @overrides
    def is_valid(self) -> bool:
        return True

    @overrides
    def motor_const(self) -> float:
        return self._motor_const.get()

    @overrides
    def moment_const(self) -> float:
        return self._moment_const.get()

    @overrides
    def rotor_drag_coef(self) -> float:
        return self._rotor_drag_coef.get()

    @overrides
    def copy_from(self, src: AerodynamicsWidget_Manual) -> None:
        super().copy_from(src)
        self._motor_const.set(src._motor_const.get())
        self._moment_const.set(src._moment_const.get())
        self._rotor_drag_coef.set(src._rotor_drag_coef.get())


class AerodynamicsWidget_BladeTheory(AerodynamicsWidget_Base):
    """Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019]"""

    NAME = "Set from blade geometry"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        abst_text = (
            "Estimate aerodynamic constants using Blade Element Theory or Momentum Theory, "
            "based on the geometric shape of the propeller set above. See "
            "<a href='https://en.wikipedia.org/wiki/Blade_element_theory'>Blade Element Theory</a> and "
            "<a href='https://en.wikipedia.org/wiki/Momentum_theory'>Momentum Theory</a> "
            "for more information."
        )
        super().__init__(main, link_name, abst_text)

    @overrides
    def is_valid(self) -> bool:
        return True

    @overrides
    def motor_const(self) -> float:
        return self._blade_thory().motor_const()

    @overrides
    def moment_const(self) -> float:
        return self._blade_thory().moment_const()

    @overrides
    def rotor_drag_coef(self) -> float:
        return self._blade_thory().rotor_drag_coef()

    @overrides
    def copy_from(self, src: AerodynamicsWidget_BladeTheory) -> None:
        super().copy_from(src)

    def _blade_thory(self) -> BladeTheory:
        blade = self._main.settings.propulsion_system.selected.get_blade_geometry(self._link_name)
        return BladeTheory(
            blade.num_blade(),
            blade.propeller_radius(),
            blade.blade_chord(),
            blade.pitch_angle(),
        )


class AerodynamicsWidget_ThrustStand(AerodynamicsWidget_Base):
    """
    推力係数とトルク係数はThrust Standの実験データから求める．\\
    空気抗力係数はBlade Theoryから求める．
    """

    NAME = "Set from thrust stand data"

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        # NOTE: テキスト中に改行コードを入れるとハイパーリンクが機能しない
        abst_text = (
            "We estimate the aerodynamic constants from data obtained through Thrust Stand experiments. "
            "For example, see the "
            "<a href='https://www.tytorobotics.com/pages/series-1580-1585'>Tyto Rootics Series 1585 Thrust Stand</a>"
        )
        super().__init__(main, link_name, abst_text)

        data_description = "Please input experimental data from the Thrust Stand."
        self._data = ParamGetterWidget_DoubleTable(
            "Data from thrust stand",
            ["RPM", "Thrust", "Torque"],
            description_text=data_description,
        )
        self._data.set_minimum([1e-1, 1e-6, 1e-6])  # TODO: 負の値にも対応
        self._data.set_decimals([0, 6, 6])
        self._data.set_suffix([" rpm", " N", " Nm"])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._rows.addWidget(self._data)

    @overrides
    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Thrust stand data is blank.")
            return False

        return True

    @overrides
    def motor_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
        data = self._data.get()
        rpm, thrust, _ = np.hsplit(data, 3)
        omega2: np.ndarray = rpm2rps(rpm) ** 2
        return ((thrust.T @ omega2) / (omega2.T @ omega2)).item()  # 最小2乗解 (memo: 2-28)

    @overrides
    def moment_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
        data = self._data.get()
        _, thrust, torque = np.hsplit(data, 3)
        return ((torque.T @ thrust) / (thrust.T @ thrust)).item()  # 最小2乗解 (memo: 2-28)

    @overrides
    def rotor_drag_coef(self) -> float:
        # FIXME: ブレードの幾何形状のみから推定するのではなく，他の空力特性を考慮して推定
        blade = self._main.settings.propulsion_system.selected.get_blade_geometry(self._link_name)
        blade_theory = BladeTheory(
            blade.num_blade(),
            blade.propeller_radius(),
            blade.blade_chord(),
            blade.pitch_angle(),
        )
        return blade_theory.rotor_drag_coef()

    @overrides
    def copy_from(self, src: AerodynamicsWidget_ThrustStand) -> None:
        super().copy_from(src)
        self._data.set(src._data.get())


class AerodynamicsWidget_UIUC(AerodynamicsWidget_Base):
    NAME = "Set from UIUC propeller data site"

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        abst_text = (
            "If the propeller is listed in the "
            "<a href='https://m-selig.ae.illinois.edu/props/propDB.html'>UIUC Propeller Data Site</a>, "
            "aerodynamic constants measured by research institutions can be utilized."
        )
        super().__init__(main, link_name, abst_text)

        data_description = "Input the Static data for the relevant propeller."
        self._data = ParamGetterWidget_DoubleTable(
            "Measurements in static condition",
            ["RPM", "CT", "CP"],
            description_text=data_description,
        )
        self._data.set_minimum([1e-3, 1e-6, 1e-6])
        self._data.set_decimals([3, 6, 6])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._rows.addWidget(self._data)

    @overrides
    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, PROPULSION_SYSTEM, "Measurements in static condition is blank.")
            return False

        return True

    @overrides
    def motor_const(self) -> float:
        # CTの平均をとる
        data = self._data.get()
        CTs = data[:, 1]
        CT = np.mean(CTs)

        blade = self._main.settings.propulsion_system.selected.get_blade_geometry(self._link_name)
        return (CT * AIR_DENSITY * blade.propeller_diameter() ** 4) / (4 * math.pi**2)

    @overrides
    def moment_const(self) -> float:
        # CT, CPの平均をとる
        # TODO: 単純な平均ではなく，ホバリング時の回転数に対応する値をとる
        data = self._data.get()
        CTs = data[:, 1]
        CPs = data[:, 2]
        CT = np.mean(CTs)
        CP = np.mean(CPs)

        blade = self._main.settings.propulsion_system.selected.get_blade_geometry(self._link_name)
        return (blade.propeller_diameter() * CP) / (2 * math.pi * CT)

    @overrides
    def rotor_drag_coef(self) -> float:
        # FIXME: ブレードの幾何形状のみから推定するのではなく，他の空力特性を考慮して推定
        blade = self._main.settings.propulsion_system.selected.get_blade_geometry(self._link_name)
        blade_theory = BladeTheory(
            blade.num_blade(),
            blade.propeller_radius(),
            blade.blade_chord(),
            blade.pitch_angle(),
        )
        return blade_theory.rotor_drag_coef()

    @overrides
    def copy_from(self, src: AerodynamicsWidget_UIUC) -> None:
        super().copy_from(src)
        self._data.set(src._data.get())
