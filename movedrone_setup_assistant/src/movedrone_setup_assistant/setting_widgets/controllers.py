from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..const import *


class ControllersWidget(BaseSettingWidget):

    LMPC_LABEL = "Linear Model Predictive Control"
    NMPC_LABEL = "Nonlinear Model Predictive Control"
    SMC_LABEL = "Model Following Sliding Mode Control"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Setup Controllers'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        type_description = "TODO: instruction"
        self.controller_type = ParamGetterWidget_ComboBox(
            "Type of Controller",
            type_description,
            [self.LMPC_LABEL, self.NMPC_LABEL, self.SMC_LABEL],
            default=self.LMPC_LABEL,
        )
        self._rows.addWidget(self.controller_type)

        self.lmpc_settings = LMPCSettingsWidget(main)
        self._rows.addWidget(self.lmpc_settings)

        self.nmpc_settings = NMPCSettingsWidget(main)
        self._rows.addWidget(self.nmpc_settings)

        self.smc_settings = SMCSettingsWidget(main)
        self._rows.addWidget(self.smc_settings)

        self._add_dummy_widget()

        self._update_controllers_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.controller_type.text_changed.connect(self._on_type_changed)

    def _update_controllers_visibility(self) -> None:
        controller_type = self.controller_type.get()

        if controller_type == self.LMPC_LABEL:
            self.lmpc_settings.setVisible(True)
            self.nmpc_settings.setVisible(False)
            self.smc_settings.setVisible(False)
        elif controller_type == self.NMPC_LABEL:
            self.lmpc_settings.setVisible(False)
            self.nmpc_settings.setVisible(True)
            self.smc_settings.setVisible(False)
        elif controller_type == self.SMC_LABEL:
            self.lmpc_settings.setVisible(False)
            self.nmpc_settings.setVisible(False)
            self.smc_settings.setVisible(True)
        else:
            raise RuntimeError(f'Invalid controller type: {controller_type}')

    @pyqtSlot(str)
    def _on_type_changed(self, controller_type: str) -> None:
        self._update_controllers_visibility()


class LMPCSettingsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = 'TODO: abstruct of LMPC'
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        natural_freq_description = "TODO: instruction"
        self.natural_freq = ParamGetterWidget_DoubleSpinBox(
            "position_controller/natural_frequency",
            natural_freq_description,
            minimum=0.1,
            default=1.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.natural_freq)

        damp_ratio_description = "TODO: instruction"
        self.damp_ratio = ParamGetterWidget_DoubleSpinBox(
            "position_controller/damping_ratio",
            damp_ratio_description,
            minimum=math.sqrt(0.5),
            default=1.,
        )
        self._rows.addWidget(self.damp_ratio)

        pred_horizon_description = "TODO: instruction"
        self.pred_horizon = ParamGetterWidget_DoubleSpinBox(
            "rotation_controller/prediction_horizon",
            pred_horizon_description,
            minimum=0.1,
            maximum=3.,
            default=1.,
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
            minimum=0.,
            maximum=1.,
            default=0.2,
        )
        self._rows.addWidget(self.rot_decay)

        angvel_decay_description = "TODO: instruction"
        self.angvel_decay = ParamGetterWidget_DoubleSpinBox(
            "rotation_controller/decay/angular_velocity",
            angvel_decay_description,
            minimum=0.,
            maximum=1.,
            default=0.,
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


class NMPCSettingsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = 'TODO: abstruct of NMPC'
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        # TODO


class SMCSettingsWidget(QWidget):
    """ モデルフォロイング型スライディングモード制御の設定(cf. 「ドローン工学入門」,p.189) """

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = 'TODO: abstruct of SMC'
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        # TODO
