from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import DoubleSpinBox

from ...parameter_getters import *
from ...common import *
from .common import STABILITY_COEF_DECIMALS


class AerodynamicsCoefficientsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel("Aerodynamic Coefficients")
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(label)

        self._form = QFormLayout()
        self._rows.addLayout(self._form)

        self.c_lift_0 = DoubleSpinBox()
        self.c_lift_0.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_lift_0.setSuffix(" [-]")
        self.c_lift_0.setValue(0.2127)
        self._form.addRow(QLabel("c_lift_0"), self.c_lift_0)

        self.c_lift_alpha = DoubleSpinBox()
        self.c_lift_alpha.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_lift_alpha.setSuffix(" [/rad]")
        self.c_lift_alpha.setValue(10.806)
        self._form.addRow(QLabel("c_lift_alpha"), self.c_lift_alpha)

        self.c_drag_0 = DoubleSpinBox()
        self.c_drag_0.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_drag_0.setSuffix(" [-]")
        self.c_drag_0.setValue(0.136)
        self._form.addRow(QLabel("c_lift_0"), self.c_drag_0)

        self.c_drag_alpha = DoubleSpinBox()
        self.c_drag_alpha.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_drag_alpha.setSuffix(" [/rad]")
        self.c_drag_alpha.setValue(0.6737)
        self._form.addRow(QLabel("c_drag_alpha"), self.c_drag_alpha)

        self.c_side_beta = DoubleSpinBox()
        self.c_side_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_side_beta.setSuffix(" [/rad]")
        self.c_side_beta.setValue(-0.3073)
        self._form.addRow(QLabel("c_side_beta"), self.c_side_beta)

        self.c_roll_beta = DoubleSpinBox()
        self.c_roll_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_roll_beta.setSuffix(" [/rad]")
        self.c_roll_beta.setValue(-0.0154)
        self._form.addRow(QLabel("c_roll_beta"), self.c_roll_beta)

        self.c_roll_p = DoubleSpinBox()
        self.c_roll_p.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_roll_p.setSuffix(" [s/rad]")
        self.c_roll_p.setValue(-0.1647)
        self._form.addRow(QLabel("c_roll_p"), self.c_roll_p)

        self.c_roll_r = DoubleSpinBox()
        self.c_roll_r.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_roll_r.setSuffix(" [s/rad]")
        self.c_roll_r.setValue(0.0117)
        self._form.addRow(QLabel("c_roll_r"), self.c_roll_r)

        self.c_pitch_0 = DoubleSpinBox()
        self.c_pitch_0.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_0.setSuffix(" [-]")
        self.c_pitch_0.setValue(0.0435)
        self._form.addRow(QLabel("c_pitch_0"), self.c_pitch_0)

        self.c_pitch_alpha = DoubleSpinBox()
        self.c_pitch_alpha.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_alpha.setSuffix(" [/rad]")
        self.c_pitch_alpha.setValue(-2.969)
        self._form.addRow(QLabel("c_pitch_alpha"), self.c_pitch_alpha)

        self.c_pitch_abs_beta = DoubleSpinBox()
        self.c_pitch_abs_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_abs_beta.setSuffix(" [/rad]")
        self.c_pitch_abs_beta.setValue(0.0)
        self._form.addRow(QLabel("c_pitch_abs_beta"), self.c_pitch_abs_beta)

        self.c_pitch_alpha_rate = DoubleSpinBox()
        self.c_pitch_alpha_rate.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_alpha_rate.setSuffix(" [s/rad]")
        self.c_pitch_alpha_rate.setValue(0.0)
        self._form.addRow(QLabel("c_pitch_alpha_rate"), self.c_pitch_alpha_rate)

        self.c_pitch_q = DoubleSpinBox()
        self.c_pitch_q.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_q.setSuffix(" [s/rad]")
        self.c_pitch_q.setValue(-106.1542)
        self._form.addRow(QLabel("c_pitch_q"), self.c_pitch_q)

        self.c_yaw_beta = DoubleSpinBox()
        self.c_yaw_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_yaw_beta.setSuffix(" [/rad]")
        self.c_yaw_beta.setValue(0.043)
        self._form.addRow(QLabel("c_yaw_beta"), self.c_yaw_beta)

        self.c_yaw_p = DoubleSpinBox()
        self.c_yaw_p.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_yaw_p.setSuffix(" [s/rad]")
        self.c_yaw_p.setValue(0.0)
        self._form.addRow(QLabel("c_yaw_p"), self.c_yaw_p)

        self.c_yaw_r = DoubleSpinBox()
        self.c_yaw_r.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_yaw_r.setSuffix(" [s/rad]")
        self.c_yaw_r.setValue(-0.0827)
        self._form.addRow(QLabel("c_yaw_r"), self.c_yaw_r)

    def define_connections(self) -> None:
        pass

    def is_valid(self) -> bool:
        return True
