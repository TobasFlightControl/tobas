from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *


class ObserverWidget(BaseSettingWidget):

    NAME = "Observer"

    NO_SELECT = "Select observer type"
    CASCADE = "Cascade Kalman Filter"
    ESKF = "Error State Kalman Filter (recommended)"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Observer"
        abst_text = "状態推定器の設定を行います．"\
            + "手法を1つ選択し，各パラメータを設定してください．"\
            + "パラメータは後からチューニングすることもできるので，デフォルトのままでも構いません．"
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

        abst_text = "この状態推定器は，姿勢推定器と位置推定器の2つの部分に分かれています．"\
            + "6軸IMUと地磁気センサの情報から相補フィルタにより姿勢を推定し，"\
            + "推定した姿勢と他のセンサの情報から線形カルマンフィルタにより3次元位置を推定します．"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        self._rows.addWidget(abst)

        gain_acc_description = "Accelerometer gain for the orientation estimation."
        self.gain_acc = ParamGetterWidget_DoubleSpinBox(
            "Acelerometer gain",
            gain_acc_description,
            decimals=3,
            minimum=0.,
            maximum=1.,
            default=0.01,
        )
        self._rows.addWidget(self.gain_acc)

        gain_mag_description = "Magnetometer gain for the orientation estimation."
        self.gain_mag = ParamGetterWidget_DoubleSpinBox(
            "Magnetometer gain",
            gain_mag_description,
            decimals=3,
            minimum=0.,
            maximum=1.,
            default=0.01,
        )
        self._rows.addWidget(self.gain_mag)

        bias_alpha_description = "Bias estimation gain for the orientation estimation."
        self.bias_alpha = ParamGetterWidget_DoubleSpinBox(
            "Bias estimation gain",
            bias_alpha_description,
            decimals=3,
            minimum=0.,
            maximum=1.,
            default=0.01,
        )
        self._rows.addWidget(self.bias_alpha)

        do_bias_estimation_description = "Whether to do bias estimation of the gyroscope readings "\
            + "for the orientation estimation."
        self.do_bias_estimation = ParamGetterWidget_CheckBox(
            "Do bias estimation",
            do_bias_estimation_description,
            check_box_text="Do bias estimation",
            default=True,
        )
        self._rows.addWidget(self.do_bias_estimation)

        do_adaptive_gain_description = "Whether to do adaptive gain for the orientation estimation."
        self.do_adaptive_gain = ParamGetterWidget_CheckBox(
            "Do adaptive gain",
            do_adaptive_gain_description,
            check_box_text="Do adaptive gain",
            default=False,
        )
        self._rows.addWidget(self.do_adaptive_gain)

        grav_var_exp_description = "The ordinary logarithm of the variance of the "\
            + "estimated gravity vector."
        self.grav_var_exp = ParamGetterWidget_SpinBox(
            "Gravity variance level",
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

        abst_text = "Quaternion kinematics for the error-state Kalman filter [Joan Sola, 2017]\n"\
            + "https://arxiv.org/abs/1711.02508"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        self._rows.addWidget(abst)

        rot_var_grav_description = "重力ベクトルの観測に用いる分散．"
        self.rot_var_grav = ParamGetterWidget_SpinBox(
            "Rotation variance (Gravity vector)",
            rot_var_grav_description,
            minimum=1,
            maximum=5000,
            default=100,
        )
        self._rows.addWidget(self.rot_var_grav)

        rot_var_geomag_description = "地磁気ベクトルの観測に用いる分散．"
        self.rot_var_geomag = ParamGetterWidget_SpinBox(
            "Rotation variance (Geomagnetic vector)",
            rot_var_geomag_description,
            minimum=1,
            maximum=5000,
            default=100,
        )
        self._rows.addWidget(self.rot_var_geomag)
