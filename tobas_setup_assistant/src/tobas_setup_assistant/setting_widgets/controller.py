from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *


class ControllerWidget(BaseSettingWidget):

    NAME = "Controller"

    NO_SELECT = "Select controller type"
    LMPC = "Linear Model Predictive Control"
    NMPC = "Nonlinear Model Predictive Control"
    SMC = "Model Following Sliding Mode Control"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Controller"
        abst_text = "飛行制御器の設定を行います．"\
            + "手法を1つ選択し，各パラメータを設定してください．"\
            + "パラメータは後からチューニングすることもできるので，デフォルトのままでも構いません．"
        super().__init__(main, title_text, abst_text)

        self.controller_type = ComboBox()
        self.controller_type.addItems([self.NO_SELECT, self.LMPC])
        # self.controller_type.addItems([self.NO_SELECT, self.LMPC, self.NMPC, self.SMC])  # TODO
        self.controller_type.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.controller_type)

        self.lmpc = ControllerWidget_LMPC(main)
        self._rows.addWidget(self.lmpc)

        self.nmpc = ControllerWidget_NMPC(main)
        self._rows.addWidget(self.nmpc)

        self.smc = ControllerWidget_SMC(main)
        self._rows.addWidget(self.smc)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.controller_type.currentTextChanged.connect(self._on_type_changed)

    def is_valid(self) -> bool:
        if self.get_type() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select controller type.")
            return False

        if self.get_type() == self.LMPC and (not self.lmpc.is_valid()):
            return False
        if self.get_type() == self.NMPC and (not self.nmpc.is_valid()):
            return False
        if self.get_type() == self.SMC and (not self.smc.is_valid()):
            return False

        return True

    def get_type(self) -> str:
        return self.controller_type.currentText()

    def pkg_name(self) -> str:
        controller_type = self.get_type()

        if controller_type == self.NO_SELECT:
            raise RuntimeError("Controller type is not selected.")
        elif controller_type == self.LMPC:
            return "tobas_multirotor_controller"
        elif controller_type == self.NMPC:
            raise NotImplementedError()
        elif controller_type == self.SMC:
            raise NotImplementedError()
        else:
            raise RuntimeError(f'Unknown controller type: {controller_type}')

    def _update_visibility(self) -> None:
        controller_type = self.get_type()

        if controller_type == self.NO_SELECT:
            self.lmpc.setVisible(False)
            self.nmpc.setVisible(False)
            self.smc.setVisible(False)
        elif controller_type == self.LMPC:
            self.lmpc.setVisible(True)
            self.nmpc.setVisible(False)
            self.smc.setVisible(False)
        elif controller_type == self.NMPC:
            self.lmpc.setVisible(False)
            self.nmpc.setVisible(True)
            self.smc.setVisible(False)
        elif controller_type == self.SMC:
            self.lmpc.setVisible(False)
            self.nmpc.setVisible(False)
            self.smc.setVisible(True)
        else:
            raise RuntimeError(f'Unknown controller type: {controller_type}')

    @pyqtSlot(str)
    def _on_type_changed(self, controller_type: str) -> None:
        self._update_visibility()


class ControllerWidget_LMPC(QWidget):

    NAME = "Linear Model Predictive Control"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "PD制御と線形モデル予測制御を組み合わせた制御器です．\n"\
            + "コマンド形式: tobas_msgs/PositionYaw.msg or tobas_msgs/VelocityYaw.msg"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

        natural_freq_description = "PD制御の自然周波数．"\
            + "大きいほど応答速度が速くなりますが，"\
            + "大きすぎると遅延やモデル化誤差などの要因により振動が発生する恐れがあります．"
        self.natural_freq = ParamGetterWidget_DoubleSpinBox(
            "Natural frequency (Position controller)",
            natural_freq_description,
            decimals=2,
            minimum=0.1,
            default=2.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.natural_freq)

        damp_ratio_description = "PD制御の減衰比．"\
            + "大きいほどオーバーシュートが小さくなりますが，"\
            + "大きすぎると速度変化に過剰に反応して振動が発生する恐れがあります．"\
            + "逆に小さいほど応答速度が速くなりますが，"\
            + "小さすぎるとオーバーシュートが大きくなり目標位置付近での振動が大きくなります．"\
            + "遅延，モデル化誤差などの無い理想的な状況では1のときに臨界減衰となり，"\
            + "オーバーシュートなく最速で目標位置に収束します．"
        self.damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "Damping ratio (Position controller)",
            damp_ratio_description,
            decimals=2,
            minimum=math.sqrt(0.5),
            default=1.,
        )
        self._rows.addWidget(self.damp_ratio)

        pred_horizon_description = "モデル予測制御の予測区間の長さ．"\
            + "理論的には大きいほど制御性能が良くなりますが，"\
            + "大きくするほど離散化誤差や計算機の数値誤差が大きくなります．"\
            + "最低でもシステム (ここでは姿勢制御) の応答時間の数倍以上にすべきだと言われています．"
        self.pred_horizon = ParamGetterWidget_DoubleSpinBox(
            "Predictin horizon (Orientation controller)",
            pred_horizon_description,
            decimals=2,
            minimum=0.1,
            maximum=3.,
            default=1.,
            suffix=" s",
        )
        self._rows.addWidget(self.pred_horizon)

        pred_steps_description = "モデル予測制御の予測区間の分割数．"\
            + "大きいほど離散化誤差が小さくなりますが，計算量は分割数の3乗に比例します．"
        self.pred_steps = ParamGetterWidget_SpinBox(
            "Prediction steps (Orientation controller)",
            pred_steps_description,
            minimum=1,
            maximum=30,
            default=10,
        )
        self._rows.addWidget(self.pred_steps)

        attitude_decay_description = "モデル予測制御における，ロール角とピッチ角の参照値の追従時定数．"\
            + "大きいほど目標値に滑らかに追従します．"
        self.attitude_decay = ParamGetterWidget_DoubleSpinBox(
            "Attitude decay time constant (Orientation controller)",
            attitude_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.1,
            suffix=" s",
        )
        self._rows.addWidget(self.attitude_decay)

        heading_decay_description = "モデル予測制御における，ヨー角の参照値の追従時定数．"\
            + "大きいほど目標値に滑らかに追従します．"
        self.heading_decay = ParamGetterWidget_DoubleSpinBox(
            "Heading decay time constant (Orientation controller)",
            heading_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.1,
            suffix=" s",
        )
        self._rows.addWidget(self.heading_decay)

        angvel_decay_description = "モデル予測制御における，角速度の参照値の追従時定数．"\
            + "大きいほど目標値に滑らかに追従します．"
        self.angvel_decay = ParamGetterWidget_DoubleSpinBox(
            "Angular velocity decay time constant (Orientation controller)",
            angvel_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.,
            suffix=" s",
        )
        self._rows.addWidget(self.angvel_decay)

        attitude_weight_description = "モデル予測制御における，ロール角とピッチ角の重み．"
        self.attitude_weight = ParamGetterWidget_SpinBox(
            "Attitude weight (Orientation controller)",
            attitude_weight_description,
            minimum=1,
            maximum=100,
            default=100,
        )
        self._rows.addWidget(self.attitude_weight)

        heading_weight_description = "モデル予測制御における，ヨー角の重み．"
        self.heading_weight = ParamGetterWidget_SpinBox(
            "Heading weight (Orientation controller)",
            heading_weight_description,
            minimum=1,
            maximum=100,
            default=10,
        )
        self._rows.addWidget(self.heading_weight)

        angvel_weight_description = "モデル予測制御における，角速度の重み．"
        self.angvel_weight = ParamGetterWidget_SpinBox(
            "Angular velocity weight (Orientation controller)",
            angvel_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.angvel_weight)

        thrust_weight_exp_description = "モデル予測制御における，プロペラ推力の重みの常用対数．"
        self.thrust_weight_exp = ParamGetterWidget_SpinBox(
            "Thrust weight level (Orientation controller)",
            thrust_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_weight_exp)

        thrust_rate_weight_exp_description = "モデル予測制御における，プロペラ推力の変化率の重みの常用対数．"
        self.thrust_rate_weight_exp = ParamGetterWidget_SpinBox(
            "Thrust rate weight level (Orientation controller)",
            thrust_rate_weight_exp_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_rate_weight_exp)

    def is_valid(self) -> bool:
        if self.attitude_decay.get() > self.pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of attitude is greater the prediction horizon.",
            )
            return False
        if self.attitude_decay.get() > self.pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of heading is greater the prediction horizon.",
            )
            return False
        if self.attitude_decay.get() > self.pred_horizon.get():
            q_error_named(
                self._main,
                self.NAME,
                "Decay time constant of angular velocity is greater the prediction horizon.",
            )
            return False

        return True


class ControllerWidget_NMPC(QWidget):
    """ Data-Driven MPC for Quadrotors [Torrente+, 2021] """

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = ""  # TODO
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

        # TODO

    def is_valid(self) -> None:
        raise NotImplementedError()  # TODO


class ControllerWidget_SMC(QWidget):
    """ モデルフォロイング型スライディングモード制御の設定(cf. 「ドローン工学入門」,p.189) """

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = ""  # TODO
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

        # TODO

    def is_valid(self) -> None:
        raise NotImplementedError()  # TODO
