from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *
from ..utils import add_expanding_widget


class ObserverWidget(BaseSettingWidget):

    NAME = "Observer"

    NO_SELECT = "Select observer type"
    CASCADE = "Cascade Kalman Filter"
    ESKF = "Error State Kalman Filter"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Observer"
        abst_text = "TODO: abstruct"
        super().__init__(main, title_text, abst_text)

        self.observer_type = ComboBox()
        self.observer_type.addItems([self.NO_SELECT, self.CASCADE, self.ESKF])
        self.observer_type.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.observer_type)

        self.cascade = ObserverWidget_Cascade(main)
        self._rows.addWidget(self.cascade)

        self.eskf = ObserverWidget_ESKF(main)
        self._rows.addWidget(self.eskf)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.observer_type.currentTextChanged.connect(self._on_type_changed)

    def is_valid(self) -> bool:
        if self.get_type() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select observer type.")
            return False

        return True

    def get_type(self) -> str:
        return self.observer_type.currentText()

    def pkg_name(self) -> str:
        observer_type = self.get_type()

        if observer_type == self.NO_SELECT:
            raise RuntimeError("Observer type is not selected.")
        elif observer_type == self.CASCADE:
            return "state_estimation_cascade"
        elif observer_type == self.ESKF:
            return "state_estimation_eskf"
        else:
            raise RuntimeError(f'Unknown observer type: {observer_type}')

    @pyqtSlot(str)
    def _on_type_changed(self, observer_type: str) -> None:
        self._update_visibility()

    def _update_visibility(self) -> None:
        observer_type = self.get_type()

        if observer_type == self.NO_SELECT:
            self.cascade.setVisible(False)
            self.eskf.setVisible(False)
        elif observer_type == self.CASCADE:
            self.cascade.setVisible(True)
            self.eskf.setVisible(False)
        elif observer_type == self.ESKF:
            self.cascade.setVisible(False)
            self.eskf.setVisible(True)
        else:
            raise RuntimeError(f'Unknown observer type: {observer_type}')


class ObserverWidget_Cascade(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "TODO: abstruct of Cascade Observer"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)

        gain_acc_description = "TODO: instruction"
        self.gain_acc = ParamGetterWidget_DoubleSpinBox(
            "orientation_estimator/gain_acc",
            gain_acc_description,
            decimals=3,
            minimum=0.,
            maximum=1.,
            default=0.01,
        )
        self._rows.addWidget(self.gain_acc)

        gain_mag_description = "TODO: instruction"
        self.gain_mag = ParamGetterWidget_DoubleSpinBox(
            "orientation_estimator/gain_mag",
            gain_mag_description,
            decimals=3,
            minimum=0.,
            maximum=1.,
            default=0.01,
        )
        self._rows.addWidget(self.gain_mag)

        bias_alpha_description = "TODO: instruction"
        self.bias_alpha = ParamGetterWidget_DoubleSpinBox(
            "orientation_estimator/bias_alpha",
            bias_alpha_description,
            decimals=3,
            minimum=0.,
            maximum=1.,
            default=0.01,
        )
        self._rows.addWidget(self.bias_alpha)

        do_bias_estimation_description = "TODO: instruction"
        self.do_bias_estimation = ParamGetterWidget_CheckBox(
            "orientation_estimator/do_bias_estimation",
            do_bias_estimation_description,
            check_box_text="Do bias estimation",
            default=True,
        )
        self._rows.addWidget(self.do_bias_estimation)

        do_adaptive_gain_description = "TODO: instruction"
        self.do_adaptive_gain = ParamGetterWidget_CheckBox(
            "orientation_estimator/do_adaptive_gain",
            do_adaptive_gain_description,
            check_box_text="Do adaptive gain",
            default=False,
        )
        self._rows.addWidget(self.do_adaptive_gain)

        grav_var_exp_description = "TODO: instruction"
        self.grav_var_exp = ParamGetterWidget_SpinBox(
            "position_estimator/gravity_variance_exp",
            grav_var_exp_description,
            minimum=-5,
            maximum=5,
            default=2,
        )
        self._rows.addWidget(self.grav_var_exp)


class ObserverWidget_ESKF(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "TODO: abstruct of ESKF"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self._rows.addWidget(abst)
