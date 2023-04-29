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
        abst_text = "TODO: abstruct"
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
            raise NotImplementedError
        elif controller_type == self.SMC:
            raise NotImplementedError
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

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "TODO: abstruct of LMPC"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        natural_freq_description = "TODO: instruction"
        self.natural_freq = ParamGetterWidget_DoubleSpinBox(
            "position_controller/natural_frequency",
            natural_freq_description,
            decimals=2,
            minimum=0.1,
            default=1.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.natural_freq)

        damp_ratio_description = "TODO: instruction"
        self.damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "position_controller/damping_ratio",
            damp_ratio_description,
            decimals=2,
            minimum=math.sqrt(0.5),
            default=1.,
        )
        self._rows.addWidget(self.damp_ratio)

        pred_horizon_description = "TODO: instruction"
        self.pred_horizon = ParamGetterWidget_DoubleSpinBox(
            "rotation_controller/prediction_horizon",
            pred_horizon_description,
            decimals=2,
            minimum=0.1,
            maximum=3.,
            default=1.,
            suffix=" s",
        )
        self._rows.addWidget(self.pred_horizon)

        pred_steps_description = "TODO: instruction"
        self.pred_steps = ParamGetterWidget_SpinBox(
            "rotation_controller/prediction_steps",
            pred_steps_description,
            minimum=1,
            maximum=30,
            default=10,
        )
        self._rows.addWidget(self.pred_steps)

        rot_decay_description = "TODO: instruction"
        self.rot_decay = ParamGetterWidget_DoubleSpinBox(
            "rotation_controller/decay/rotation",
            rot_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.2,
            suffix=" s",
        )
        self._rows.addWidget(self.rot_decay)

        angvel_decay_description = "TODO: instruction"
        self.angvel_decay = ParamGetterWidget_DoubleSpinBox(
            "rotation_controller/decay/angular_velocity",
            angvel_decay_description,
            decimals=2,
            minimum=0.,
            maximum=1.,
            default=0.,
            suffix=" s",
        )
        self._rows.addWidget(self.angvel_decay)

        rot_weight_description = "TODO: instruction"
        self.rot_weight = ParamGetterWidget_SpinBox(
            "rotation_controller/weight/rotation",
            rot_weight_description,
            minimum=1,
            maximum=100,
            default=100,
        )
        self._rows.addWidget(self.rot_weight)

        angvel_weight_description = "TODO: instruction"
        self.angvel_weight = ParamGetterWidget_SpinBox(
            "rotation_controller/weight/angular_velocity",
            angvel_weight_description,
            minimum=1,
            maximum=100,
            default=1,
        )
        self._rows.addWidget(self.angvel_weight)

        thrust_weight_description = "TODO: instruction"
        self.thrust_weight = ParamGetterWidget_SpinBox(
            "rotation_controller/weight/thrust_force",
            thrust_weight_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_weight)

        thrust_rate_weight_description = "TODO: instruction"
        self.thrust_rate_weight = ParamGetterWidget_SpinBox(
            "rotation_controller/weight/thrust_force_rate",
            thrust_rate_weight_description,
            minimum=-6,
            maximum=0,
            default=-3,
        )
        self._rows.addWidget(self.thrust_rate_weight)


class ControllerWidget_NMPC(QWidget):
    """ Data-Driven MPC for Quadrotors [Torrente+, 2021] """

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "TODO: abstruct of NMPC"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        # TODO


class ControllerWidget_SMC(QWidget):
    """ モデルフォロイング型スライディングモード制御の設定(cf. 「ドローン工学入門」,p.189) """

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "TODO: abstruct of SMC"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        # TODO
