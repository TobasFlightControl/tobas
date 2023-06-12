from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .esc import EscWidget_Base

import math
from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...constants import *
from .constants import ROTARY_WINGS


class AerodynamicsWidget(QWidget):

    NO_SELECT = "Select setting method"
    MANUAL = "Set manually"
    BLADE_THEORY = "Set from rough blade shape"
    THRUST_STAND = "Set from thrust stand data (recommended)"

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
            [self.NO_SELECT, self.MANUAL, self.BLADE_THEORY, self.THRUST_STAND]
        )
        self.setting_method.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.setting_method)

        self.manual = AerodynamicsWidget_Manual(main, link_name)
        self._rows.addWidget(self.manual)

        self.blade_theory = AerodynamicsWidget_BladeTheory(main, link_name)
        self._rows.addWidget(self.blade_theory)

        self.thrust_stand = AerodynamicsWidget_ThrustStand(main, link_name)
        self._rows.addWidget(self.thrust_stand)

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

    def selected(self) -> EscWidget_Base:
        setting_method = self.setting_method.currentText()

        if setting_method == self.MANUAL:
            return self.manual
        elif setting_method == self.BLADE_THEORY:
            return self.blade_theory
        elif setting_method == self.THRUST_STAND:
            return self.thrust_stand
        else:
            raise RuntimeError

    def motor_const(self) -> float:
        """ [kg*m/s^2] """
        return self._selected_setting_widget().motor_const()

    def moment_const(self) -> float:
        """ [m] """
        return self._selected_setting_widget().moment_const()

    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        return self._selected_setting_widget().rotor_drag_coef()

    def copy_from(self, src: AerodynamicsWidget) -> None:
        self.setting_method.setCurrentText(src.setting_method.currentText())
        self.manual.copy_from(src.manual)
        self.blade_theory.copy_from(src.blade_theory)
        self.thrust_stand.copy_from(src.thrust_stand)

        self._update_visibility()

    def _define_connections(self) -> None:
        self.setting_method.currentTextChanged.connect(self._on_type_changed)

    def _update_visibility(self) -> None:
        setting_method = self.setting_method.currentText()

        if setting_method == self.NO_SELECT:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(False)
        elif setting_method == self.MANUAL:
            self.manual.setVisible(True)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(False)
        elif setting_method == self.BLADE_THEORY:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(True)
            self.thrust_stand.setVisible(False)
        elif setting_method == self.THRUST_STAND:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.thrust_stand.setVisible(True)
        else:
            raise RuntimeError(f'Unknown setting method: {setting_method}')

    def _selected_setting_widget(self) -> AerodynamicsWidget_Base:
        setting_method = self.setting_method.currentText()

        if setting_method == self.MANUAL:
            return self.manual
        elif setting_method == self.BLADE_THEORY:
            return self.blade_theory
        elif setting_method == self.THRUST_STAND:
            return self.thrust_stand
        else:
            raise RuntimeError

    @pyqtSlot(str)
    def _on_type_changed(self, setting_method: str) -> None:
        self._update_visibility()


class AerodynamicsWidget_Base(QWidget):

    rho = 1.225         # Air density [kg/m^3]
    a = 2 * math.pi     # 2D lift curve slope (ideal value)
    B = 0.9             # Tip loss factor
    gamma = 8.          # Lock number (typical value, cf. Balic Helicopter Aerodynamics p.66)
    C_d0 = 0.02         # Profile drag coefficient (typical value)

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def motor_const(self) -> float:
        """ [kg*m/s^2] """
        raise NotImplementedError

    @abstractmethod
    def moment_const(self) -> float:
        """ [m] """
        raise NotImplementedError

    @abstractmethod
    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        raise NotImplementedError

    @abstractmethod
    def copy_from(self, src) -> None:
        raise NotImplementedError


class AerodynamicsWidget_Manual(AerodynamicsWidget_Base):

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

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
        super().__init__(main, link_name)

        num_blade_description = "1つのプロペラに対するブレードの個数．"
        self._num_blade = ParamGetterWidget_SpinBox(
            "Number of blades",
            num_blade_description,
            minimum=1,
            default=2,
        )
        self._rows.addWidget(self._num_blade)

        rotor_radius_description = "プロペラの回転面の半径．つまり回転中心からブレードの先端までの距離．"
        self._rotor_radius = ParamGetterWidget_SpinBox(
            "Rotor radius",
            rotor_radius_description,
            minimum=1,
            default=100,
            suffix=" mm",
        )
        self._rows.addWidget(self._rotor_radius)

        blade_chord_description = "ブレードの弦長．"\
            + "測定位置によって異なる場合，元論文ではブレードの長さの75%の位置で計測しているようです．"
        self._blade_chord = ParamGetterWidget_SpinBox(
            "Blade chord",
            blade_chord_description,
            minimum=1,
            default=15,
            suffix=" mm",
        )
        self._rows.addWidget(self._blade_chord)

        pitch_avg_description = "ブレードの平均ねじれ角．"
        self._pitch_avg = ParamGetterWidget_SpinBox(
            "Blade average pitch angle",
            pitch_avg_description,
            minimum=1,
            maximum=90,
            default=10,
            suffix=" deg",
        )
        self._rows.addWidget(self._pitch_avg)

    def is_valid(self) -> bool:
        return True

    def motor_const(self) -> float:
        return 4 * math.pi * self._C_T() * self.rho * self._R()**4

    def moment_const(self) -> float:
        return self._R() * self._lambda()

    def rotor_drag_coef(self) -> float:
        return 4 * math.pi * self.rho * self._R()**3 * self._C_H()

    def copy_from(self, src: AerodynamicsWidget_BladeTheory) -> None:
        self._num_blade.set(src._num_blade.get())
        self._rotor_radius.set(src._rotor_radius.get())
        self._blade_chord.set(src._blade_chord.get())
        self._pitch_avg.set(src._pitch_avg.get())

    def _N(self) -> int:
        """ Number of blades """
        return self._num_blade.get()

    def _R(self) -> float:
        """ Rotor radius [m] """
        return self._rotor_radius.get() / 1000.

    def _c(self) -> float:
        """ Blade chord [m] """
        return self._blade_chord.get() / 1000.

    def _theta(self) -> float:
        """ Blade average pitch angle [rad] """
        return math.radians(self._pitch_avg.get())

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
        b1s = -(4/3) * b0                   # devided by mu
        return (sigma / 4) * (self.C_d0 + (self.a / 6) * (2 * theta * (3 * lam - 2 * b1c) +
                                                          9 * lam * b1c + 2 * b0 * b1s + 3 * b0**2))


class AerodynamicsWidget_ThrustStand(AerodynamicsWidget_Base):
    """
    推力係数とトルク係数はThrust Standの実験データから求める．\\
    空気抗力係数は上の係数とBlade Theoryから求める．
    """

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        # 同じ計算を繰り返すことを防ぐためにMotor ConstとMoment Constをキャッシュする
        self._motor_const = -1.
        self._moment_const = -1.
        self._motor_const_updated = False
        self._moment_const_updated = False

        num_blade_description = "1つのプロペラに対するブレードの個数．"
        self._num_blade = ParamGetterWidget_SpinBox(
            "Number of blades",
            num_blade_description,
            minimum=1,
            default=2,
        )
        self._rows.addWidget(self._num_blade)

        blade_chord_description = "ブレードの弦長．"\
            + "測定位置によって異なる場合，元論文ではブレードの長さの75%の位置で計測しているようです．"
        self._blade_chord = ParamGetterWidget_SpinBox(
            "Blade chord",
            blade_chord_description,
            minimum=1,
            default=15,
            suffix=" mm",
        )
        self._rows.addWidget(self._blade_chord)

        data_description = "Thrust Stand実験のデータから，シミュレーションと制御に必要な空力定数を推定します．"\
            + "データを直接入力するか，CSVファイルを読み込んでください．\n"\
            + "Thrust Standの例: https://www.tytorobotics.com/pages/series-1580-1585"
        self._data = ParamGetterWidget_DoubleTable(
            "Data from thrust stand",
            ["Rotation Speed", "Thrust", "Torque"],
            description_text=data_description,
        )
        self._data.set_minimum([1e-1, 1e-6, 1e-6])  # TODO: 負の値にも対応
        self._data.set_decimals([1, 6, 6])
        self._data.set_suffix([" rpm", " N", " Nm"])
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._rows.addWidget(self._data)

        self._data.data_changed.connect(self._on_data_changed)

    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, ROTARY_WINGS, "Thrust stand data is blank.")
            return False

        return True

    def motor_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデル(２次関数)からかけ離れていたら警告を出す

        # Motor Constが更新されていなければ更新
        if not self._motor_const_updated:
            data = self._data.get()
            num_samples = data.shape[0]
            assert num_samples > 0

            motor_const_sum = 0.
            for omega, thrust, _ in data:
                motor_const = thrust / omega**2
                motor_const_sum += motor_const

            self._motor_const = motor_const_sum / num_samples
            self._motor_const_updated = True

        return self._motor_const

    def moment_const(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりにモデルからかけ離れていたら警告を出す

        # Moment Constが更新されていなければ更新
        if not self._moment_const_updated:
            data = self._data.get()
            num_samples = data.shape[0]
            assert num_samples > 0

            moment_const_sum = 0.
            for omega, thrust, torque in data:
                motor_const = thrust / omega**2
                moment_const = torque / motor_const
                moment_const_sum += moment_const

            self._moment_const = moment_const_sum / num_samples
            self._moment_const_updated = True

        return self._moment_const

    def rotor_drag_coef(self) -> float:
        return float(4 * math.pi * self.rho * self._R()**3 * self._C_H())

    def copy_from(self, src: AerodynamicsWidget_ThrustStand) -> None:
        self._num_blade.set(src._num_blade.get())
        self._blade_chord.set(src._blade_chord.get())
        self._data.set(src._data.get())

    def _N(self) -> int:
        """ Number of blades """
        return self._num_blade.get()

    def _c(self) -> float:
        """ Measured blade chord [m] """
        return self._blade_chord.get() / 1000.

    def _R(self) -> float:
        """ Estimated rotor radius [m] """
        return math.sqrt(self.motor_const() / (8 * math.pi * self.rho)) / self.moment_const()

    def _theta(self) -> float:
        """ Estimated blade average pitch angle [rad] """
        lam = self._lambda()
        return (3 * lam) / (2 * self.B) * (1 + (8 * lam) / (self.B**2 * self._sigma() * self.a))

    def _sigma(self) -> float:
        """ Solidity """
        return (self._N() * self._c()) / (math.pi * self._R())

    def _lambda(self) -> float:
        """ Inflow ratio """
        return self.moment_const()**2 * math.sqrt(8 * math.pi * self.rho / self.motor_const())

    def _C_H(self) -> float:
        """ Horizontal force coefficient (devided by mu) """
        theta = self._theta()
        sigma = self._sigma()
        lam = self._lambda()
        b0 = 0.5 * self.gamma * (theta / 4 - lam / 3)
        b1c = 2 * (lam - (4 / 3) * theta)   # devided by mu
        b1s = -(4/3) * b0                   # devided by mu
        return (sigma / 4) * (self.C_d0 + (self.a / 6) * (2 * theta * (3 * lam - 2 * b1c) +
                                                          9 * lam * b1c + 2 * b0 * b1s + 3 * b0**2))

    @pyqtSlot()
    def _on_data_changed(self) -> None:
        self._motor_const_updated = False
        self._moment_const_updated = False
