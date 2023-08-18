from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .blade_geometry import BladeGeometry

import math
import numpy as np
from numpy.typing import NDArray
from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from ...utils import rpm_to_rad_per_sec
from .common import ROTARY_WINGS


class AerodynamicsWidget(QWidget):

    NO_SELECT = "Select setting method"
    MANUAL = "Set manually"
    BLADE_THEORY = "Set from blade geometry"
    THRUST_STAND = "Set from thrust stand data"
    UIUC = "Set from UIUC propeller data site (recommended)"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel("Aerodynamics")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self.setting_method = ComboBox()
        self.setting_method.addItems(
            [self.NO_SELECT, self.MANUAL, self.BLADE_THEORY, self.THRUST_STAND, self.UIUC]
        )
        self.setting_method.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.setting_method)

        self.manual = AerodynamicsWidget_Manual(main, link_name)
        self._rows.addWidget(self.manual)

        self.blade_theory = AerodynamicsWidget_BladeTheory(main, link_name)
        self._rows.addWidget(self.blade_theory)

        self.thrust_stand = AerodynamicsWidget_ThrustStand(main, link_name)
        self._rows.addWidget(self.thrust_stand)

        self.uiuc = AerodynamicsWidget_UIUC(main, link_name)
        self._rows.addWidget(self.uiuc)

        self._update_visibility()
        self._define_connections()

    def is_valid(self) -> bool:
        if self.setting_method.currentText() == self.NO_SELECT:
            q_error_named(self._main, ROTARY_WINGS, "Please select aerodynamics setting method.")
            return False
        else:
            if not self.selected().is_valid():
                return False

        return True

    def selected(self) -> AerodynamicsWidget:
        setting_method = self.setting_method.currentText()

        if setting_method == self.MANUAL:
            return self.manual
        elif setting_method == self.BLADE_THEORY:
            return self.blade_theory
        elif setting_method == self.THRUST_STAND:
            return self.thrust_stand
        elif setting_method == self.UIUC:
            return self.uiuc
        else:
            raise RuntimeError()

    def motor_const(self) -> float:
        """ [kg*m/s^2] """
        return self.selected().motor_const()

    def moment_const(self) -> float:
        """ [m] """
        return self.selected().moment_const()

    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        return self.selected().rotor_drag_coef()

    def copy_from(self, src: AerodynamicsWidget) -> None:
        self.setting_method.setCurrentText(src.setting_method.currentText())
        self.manual.copy_from(src.manual)
        self.blade_theory.copy_from(src.blade_theory)
        self.thrust_stand.copy_from(src.thrust_stand)
        self.uiuc.copy_from(src.uiuc)

        self._update_visibility()

    def _define_connections(self) -> None:
        self.setting_method.currentTextChanged.connect(self._on_type_changed)

    def _update_visibility(self) -> None:
        setting_method = self.setting_method.currentText()

        if setting_method == self.NO_SELECT:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(False)
            self.uiuc.setVisible(False)
        elif setting_method == self.MANUAL:
            self.manual.setVisible(True)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(False)
            self.uiuc.setVisible(False)
        elif setting_method == self.BLADE_THEORY:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(True)
            self.thrust_stand.setVisible(False)
            self.uiuc.setVisible(False)
        elif setting_method == self.THRUST_STAND:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(True)
            self.uiuc.setVisible(False)
        elif setting_method == self.UIUC:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(False)
            self.uiuc.setVisible(True)
        else:
            raise RuntimeError(f'Unknown setting method: {setting_method}')

    @pyqtSlot(str)
    def _on_type_changed(self, setting_method: str) -> None:
        self._update_visibility()


class AerodynamicsWidget_Base(QWidget):

    rho = 1.225         # Air density [kg/m^3]
    a = 2 * math.pi     # 2D lift curve slope (ideal value)
    B = 0.9             # Tip loss factor
    gamma = 8.          # Lock number (typical value, cf. Balic Helicopter Aerodynamics p.66)
    C_d0 = 0.02         # Profile drag coefficient (typical value)

    def __init__(self, main: SetupAssistant, link_name: str, abst_text: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def motor_const(self) -> float:
        """ [kg*m/s^2] """
        raise NotImplementedError()

    @abstractmethod
    def moment_const(self) -> float:
        """ [m] """
        raise NotImplementedError()

    @abstractmethod
    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        raise NotImplementedError()

    @abstractmethod
    def copy_from(self, src) -> None:
        raise NotImplementedError()


class AerodynamicsWidget_Manual(AerodynamicsWidget_Base):

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        abst_text = "プロペラ空気力学定数を直接設定します．"
        super().__init__(main, link_name, abst_text)

        motor_const_description = "プロペラの推力定数．"\
            + "推力定数をc[kg*m/rad^2]，回転数をw[rad/s]とすると，回転面に垂直な向きに発生する推力T[N]は"\
            + " T = c w^2 と表されます．"
        self._motor_const = ParamGetterWidget_DoubleSpinBox(
            "Motor Constant",
            motor_const_description,
            decimals=12,
            minimum=0.,
            default=8.54858e-6,
            suffix=" kg*m/rad^2",
        )
        self._rows.addWidget(self._motor_const)

        moment_const_description = "プロペラの反トルク係数．"\
            + "反トルク係数をc[m]，プロペラの推力をT[N]とすると，プロペラの回転方向と逆向きに発生するトルク[Nm]は"\
            + " N = c T と表されます．"
        self._moment_const = ParamGetterWidget_DoubleSpinBox(
            "Moment Constant",
            moment_const_description,
            decimals=6,
            minimum=0.,
            default=0.016,
            suffix=" m",
        )
        self._rows.addWidget(self._moment_const)

        rotor_drag_coef_description = "プロペラの空気抗力係数．"\
            + "空気抗力係数をc[kg/rad]，モータの回転数をw[rad/s]，"\
            + "機体に対する相対的な大気速度の回転軸に垂直な成分の大きさをV[m/s]とすると，"\
            + "プロペラに発生する空気抗力の大きさF[N]は F = c w V と表されます．"
        self._rotor_drag_coef = ParamGetterWidget_DoubleSpinBox(
            "Rotor Drag Coefficient",
            rotor_drag_coef_description,
            decimals=9,
            minimum=0.,
            default=8.06428e-5,
            suffix=" kg/rad",
        )
        self._rows.addWidget(self._rotor_drag_coef)

    def is_valid(self) -> bool:
        return True

    def motor_const(self) -> float:
        return self._motor_const.get()

    def moment_const(self) -> float:
        return self._moment_const.get()

    def rotor_drag_coef(self) -> float:
        return self._rotor_drag_coef.get()

    def copy_from(self, src: AerodynamicsWidget_Manual) -> None:
        self._motor_const.set(src._motor_const.get())
        self._moment_const.set(src._moment_const.get())
        self._rotor_drag_coef.set(src._rotor_drag_coef.get())


class AerodynamicsWidget_BladeTheory(AerodynamicsWidget_Base):
    """ Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019] """

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        abst_text = "上で設定したプロペラの幾何形状から"\
            + "<a href='https://en.wikipedia.org/wiki/Blade_element_theory'>Blade Element Theory</a>や"\
            + "<a href='https://en.wikipedia.org/wiki/Momentum_theory'>Momentum Theory</a>"\
            + "を利用して空力定数を推定します．"
        super().__init__(main, link_name, abst_text)

    def is_valid(self) -> bool:
        return True

    def motor_const(self) -> float:
        return 4 * math.pi * self._C_T() * self.rho * self._R()**4

    def moment_const(self) -> float:
        return self._R() * self._lambda()

    def rotor_drag_coef(self) -> float:
        return 4 * math.pi * self.rho * self._R()**3 * self._C_H()

    def copy_from(self, src: AerodynamicsWidget_BladeTheory) -> None:
        pass

    def _blade(self) -> BladeGeometry:
        return self._main.settings.rotary_wings.selected.get_blade_geometry(self._link_name)

    def _N(self) -> int:
        """ Number of blades """
        return self._blade().num_blade()

    def _R(self) -> float:
        """ Rotor radius [m] """
        return self._blade().propeller_radius()

    def _c(self) -> float:
        """ Blade chord [m] """
        return self._blade().blade_chord()

    def _theta(self) -> float:
        """ Blade pitch angle [rad] """
        return self._blade().pitch_angle()

    def _sigma(self) -> float:
        """ Solidity """
        return (self._N() * self._c()) / (math.pi * self._R())

    def _lambda(self) -> float:
        """ Inflow ratio """
        a_B_sigma = self.a * self.B * self._sigma()
        return a_B_sigma * self.B / 16 * (math.sqrt(1 + (64 * self._theta()) / (3 * a_B_sigma)) - 1)

    def _C_T(self) -> float:
        """ Thrust coefficient """
        return 2 * self._lambda()**2

    def _C_H(self) -> float:
        """ Horizontal force coefficient (devided by mu) """
        theta = self._theta()
        sigma = self._sigma()
        lam = self._lambda()
        b0 = 0.5 * self.gamma * (theta / 4 - lam / 3)
        b1c = 2 * (lam - (4 / 3) * theta)   # devided by mu
        b1s = -(4 / 3) * b0                 # devided by mu
        return (sigma / 4) * (self.C_d0 + (self.a / 6) * (2 * theta * (3 * lam - 2 * b1c) +
                                                          9 * lam * b1c + 2 * b0 * b1s + 3 * b0**2))


class AerodynamicsWidget_ThrustStand(AerodynamicsWidget_Base):
    """
    推力係数とトルク係数はThrust Standの実験データから求める．\\
    空気抗力係数はBlade Theoryから求める．
    """

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        # FIXME: テキスト中に改行コードを入れるとハイパーリンクが機能しない
        abst_text = "Thrust Stand実験のデータから，空力定数を推定します．"\
            + "Thrust Standの例: <a href='https://www.tytorobotics.com/pages/series-1580-1585'>"\
            + "Tyto Rootics Series 1585 Thrust Stand</a>"
        super().__init__(main, link_name, abst_text)

        data_description = "Thrust Standの実験データを入力してください．"
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

    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, ROTARY_WINGS, "Thrust stand data is blank.")
            return False

        return True

    def motor_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
        data = self._data.get()
        rpm, thrust, _ = np.hsplit(data, 3)
        omega2: NDArray = rpm_to_rad_per_sec(rpm)**2
        return ((thrust.T @ omega2) / (omega2.T @ omega2)).item()  # 最小2乗解 (memo: 2-28)

    def moment_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
        data = self._data.get()
        _, thrust, torque = np.hsplit(data, 3)
        return ((torque.T @ thrust) / (thrust.T @ thrust)).item()  # 最小2乗解 (memo: 2-28)

    def rotor_drag_coef(self) -> float:
        # Blade Theoryと同じ計算方法
        aero_dynamics = self._main.settings.rotary_wings.selected.get_aerodynamics(self._link_name)
        return aero_dynamics.blade_theory.rotor_drag_coef()

    def copy_from(self, src: AerodynamicsWidget_ThrustStand) -> None:
        self._data.set(src._data.get())


class AerodynamicsWidget_UIUC(AerodynamicsWidget_Base):

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        abst_text = "プロペラが"\
            + "<a href='https://m-selig.ae.illinois.edu/props/propDB.html'>"\
            + "UIUC Propeller Data Site</a>"\
            + "に登録されている場合は，研究機関によって測定された空力定数が利用できます．"
        super().__init__(main, link_name, abst_text)

        data_description = "該当するプロペラのStaticデータを入力してください．"
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

    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, ROTARY_WINGS, "Measurements in static condition is blank.")
            return False

        return True

    def motor_const(self) -> float:
        # CTの平均をとる
        data = self._data.get()
        CTs = data[:, 1]
        CT = np.mean(CTs)

        blade = self._main.settings.rotary_wings.selected.get_blade_geometry(self._link_name)
        return (CT * self.rho * blade.propeller_diameter()**4) / (4 * math.pi**2)

    def moment_const(self) -> float:
        # CT, CPの平均をとる
        # TODO: 単純な平均ではなく，ホバリング時の回転数に対応する値をとる
        data = self._data.get()
        CTs = data[:, 1]
        CPs = data[:, 2]
        CT = np.mean(CTs)
        CP = np.mean(CPs)

        blade = self._main.settings.rotary_wings.selected.get_blade_geometry(self._link_name)
        return (blade.propeller_diameter() * CP) / (2 * math.pi * CT)

    def rotor_drag_coef(self) -> float:
        # Blade Theoryと同じ計算方法
        aero_dynamics = self._main.settings.rotary_wings.selected.get_aerodynamics(self._link_name)
        return aero_dynamics.blade_theory.rotor_drag_coef()

    def copy_from(self, src: AerodynamicsWidget_UIUC) -> None:
        self._data.set(src._data.get())
