from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..common import *


class ObserverWidget(BaseSettingWidget):
    NAME = "Observer"

    NO_SELECT = "Select observer type"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Observer"
        abst_text = (
            "状態推定器の設定を行います．"
            + "手法を1つ選択し，各パラメータを設定してください．"
            + "パラメータは後からチューニングすることもできるので，デフォルトのままでも構いません．"
        )
        super().__init__(main, title_text, abst_text)

        self.type = ComboBox()
        self.type.addItem(self.NO_SELECT)
        self.type.addItem(ObserverWidget_Cascade.NAME)
        self.type.addItem(ObserverWidget_ESKF.NAME)
        self.type.setCurrentText(ObserverWidget_ESKF.NAME)
        self._rows.addWidget(self.type)

        self.cascade = ObserverWidget_Cascade(main)
        self._rows.addWidget(self.cascade)

        self.eskf = ObserverWidget_ESKF(main)
        self._rows.addWidget(self.eskf)

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self.type.currentTextChanged.connect(self._on_type_changed)

    @overrides
    def is_valid(self) -> bool:
        if self.get_type() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select observer type.")
            return False

        if not self.selected().is_valid():
            return False

        return True

    def selected(self) -> ObserverWidget_Base:
        observer_type = self.get_type()

        if observer_type == self.NO_SELECT:
            raise RuntimeError("Observer type is not selected.")
        elif observer_type == ObserverWidget_Cascade.NAME:
            return self.cascade
        elif observer_type == ObserverWidget_ESKF.NAME:
            return self.eskf
        else:
            raise RuntimeError(f"Unknown observer type: {observer_type}")

    def get_type(self) -> str:
        return self.type.currentText()

    def pkg_name(self) -> str:
        return self.selected().PACKAGE_NAME

    @pyqtSlot(str)
    def _on_type_changed(self, observer_type: str) -> None:
        self._update_visibility()

    def _update_visibility(self) -> None:
        observer_type = self.get_type()

        if observer_type == self.NO_SELECT:
            self.cascade.setVisible(False)
            self.eskf.setVisible(False)
        elif observer_type == ObserverWidget_Cascade.NAME:
            self.cascade.setVisible(True)
            self.eskf.setVisible(False)
        elif observer_type == ObserverWidget_ESKF.NAME:
            self.cascade.setVisible(False)
            self.eskf.setVisible(True)
        else:
            raise RuntimeError(f"Unknown observer type: {observer_type}")


class ObserverWidget_Base(QWidget):
    NAME = "Unknown"
    PACKAGE_NAME = "Unknown"

    def __init__(self, main: SetupAssistant, abst_text: str) -> None:
        super().__init__()

        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        abst.setOpenExternalLinks(True)
        self._rows.addWidget(abst)

        gps_hor_pos_stddev_threshold_description = (
            "GPSを用いて初期位置合わせをする際の，"
            + "水平位置の真値に対する標準偏差の閾値．"
            + "小さいほど初期位置を精度良く求めるが，位置合わせにかかる時間が増える．"
        )
        self.gps_hor_pos_stddev_threshold = ParamGetterWidget_DoubleSpinBox(
            "GPS horizontal position std. dev threshold",
            gps_hor_pos_stddev_threshold_description,
            decimals=2,
            minimum=0.01,
            maximum=1.0,
            default=0.3,
            suffix=" m",
        )
        self._rows.addWidget(self.gps_hor_pos_stddev_threshold)

        gps_ver_pos_stddev_threshold_description = (
            "GPSを用いて初期位置合わせをする際の，"
            + "垂直位置の真値に対する標準偏差の閾値．"
            + "小さいほど初期位置を精度良く求めるが，位置合わせにかかる時間が増える．"
        )
        self.gps_ver_pos_stddev_threshold = ParamGetterWidget_DoubleSpinBox(
            "GPS vertical position std. dev threshold",
            gps_ver_pos_stddev_threshold_description,
            decimals=2,
            minimum=0.01,
            maximum=1.0,
            default=0.6,
            suffix=" m",
        )
        self._rows.addWidget(self.gps_ver_pos_stddev_threshold)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def parameter_dict(self) -> dict:
        raise NotImplementedError()


class ObserverWidget_Cascade(ObserverWidget_Base):
    NAME = "Cascade Kalman Filter"
    PACKAGE_NAME = "state_estimation_cascade"

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = (
            "この状態推定器は，姿勢推定器と位置推定器の2つの部分に分かれています．"
            + "6軸IMUと地磁気センサの情報から相補フィルタにより姿勢を推定し，"
            + "推定した姿勢と他のセンサの情報から線形カルマンフィルタにより3次元位置を推定します．"
        )
        super().__init__(main, abst_text)

        gain_acc_description = "Accelerometer gain for the orientation estimation."
        self._gain_acc = ParamGetterWidget_DoubleSpinBox(
            "Acelerometer gain",
            gain_acc_description,
            decimals=3,
            minimum=0.0,
            maximum=1.0,
            default=0.01,
        )
        self._rows.addWidget(self._gain_acc)

        gain_mag_description = "Magnetometer gain for the orientation estimation."
        self._gain_mag = ParamGetterWidget_DoubleSpinBox(
            "Magnetometer gain",
            gain_mag_description,
            decimals=3,
            minimum=0.0,
            maximum=1.0,
            default=0.01,
        )
        self._rows.addWidget(self._gain_mag)

        bias_alpha_description = "Bias estimation gain for the orientation estimation."
        self.bias_alpha = ParamGetterWidget_DoubleSpinBox(
            "Bias estimation gain",
            bias_alpha_description,
            decimals=3,
            minimum=0.0,
            maximum=1.0,
            default=0.01,
        )
        self._rows.addWidget(self.bias_alpha)

        do_bias_estimation_description = (
            "Whether to do bias estimation of the gyroscope readings "
            + "for the orientation estimation."
        )
        self._do_bias_estimation = ParamGetterWidget_CheckBox(
            "Do bias estimation",
            do_bias_estimation_description,
            check_box_text="Do bias estimation",
            default=True,
        )
        self._rows.addWidget(self._do_bias_estimation)

        do_adaptive_gain_description = (
            "Whether to do adaptive gain for the orientation estimation."
        )
        self._do_adaptive_gain = ParamGetterWidget_CheckBox(
            "Do adaptive gain",
            do_adaptive_gain_description,
            check_box_text="Do adaptive gain",
            default=False,
        )
        self._rows.addWidget(self._do_adaptive_gain)

        grav_var_description = "The process noise variance of the gravity vector."
        self._grav_var = ParamGetterWidget_SpinBox(
            "Gravity variance",
            grav_var_description,
            minimum=1,
            maximum=1000,
            default=100,
        )
        self._rows.addWidget(self._grav_var)

    @overrides
    def is_valid(self) -> bool:
        # 絶対位置が取得できないとダメ
        no_gps = not self._main.settings.gps.equipped()
        no_odom = not self._main.settings.odometry.equipped()
        if no_gps and no_odom:
            q_error_named(
                self._main,
                self.NAME,
                "Absolute position connot be observed. Please review the sensor settings",
            )
            return False

        return True

    @overrides
    def parameter_dict(self) -> dict:
        res = dict()
        res["orientation_estimator_complement"] = {
            "gain_acc": self._gain_acc.get(),
            "gain_mag": self._gain_mag.get(),
            "bias_alpha": self.bias_alpha.get(),
            "do_bias_estimation": self._do_bias_estimation.get(),
            "do_adaptive_gain": self._do_adaptive_gain.get(),
        }
        res["state_estimator_cascade"] = {
            "use_gps": self._main.settings.gps.equipped(),
            "gps_horizontal_position_stddev_threshold": self.gps_hor_pos_stddev_threshold.get(),
            "gps_vertical_position_stddev_threshold": self.gps_ver_pos_stddev_threshold.get(),
            "gravity_variance": self._grav_var.get(),
        }

        return res


class ObserverWidget_ESKF(ObserverWidget_Base):
    NAME = "Error State Kalman Filter"
    PACKAGE_NAME = "state_estimation_eskf"

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = (
            "An implementation of <a href='https://arxiv.org/abs/1711.02508'>"
            + "Quaternion kinematics for the error-state Kalman filter [Joan Sola, 2017]</a>."
        )
        super().__init__(main, abst_text)

        rot_var_grav_description = "重力ベクトルの観測に用いる分散．"
        self._rot_var_grav = ParamGetterWidget_SpinBox(
            "Rotation variance (Gravity vector)",
            rot_var_grav_description,
            minimum=1,
            maximum=5000,
            default=100,
        )
        self._rows.addWidget(self._rot_var_grav)

        rot_var_geomag_description = "地磁気ベクトルの観測に用いる分散．"
        self._rot_var_geomag = ParamGetterWidget_SpinBox(
            "Rotation variance (Geomagnetic vector)",
            rot_var_geomag_description,
            minimum=1,
            maximum=5000,
            default=1,
        )
        self._rows.addWidget(self._rot_var_geomag)

        acc_bias_noise_var_description = "加速度センサバイアスの観測ノイズの分散の常用対数．"
        self._acc_bias_noise_var_log10 = ParamGetterWidget_SpinBox(
            "Accelerometer bias noise variance level",
            acc_bias_noise_var_description,
            minimum=-12,
            maximum=0,
            default=-5,
        )
        self._rows.addWidget(self._acc_bias_noise_var_log10)

        gyro_bias_noise_var_description = "加速度センサバイアスの観測ノイズの分散の常用対数．"
        self._gyro_bias_noise_var_log10 = ParamGetterWidget_SpinBox(
            "Gyroscope bias noise variance level",
            gyro_bias_noise_var_description,
            minimum=-12,
            maximum=0,
            default=-9,
        )
        self._rows.addWidget(self._gyro_bias_noise_var_log10)

    @overrides
    def is_valid(self) -> bool:
        # 絶対位置が取得できないとダメ
        no_gps = not self._main.settings.gps.equipped()
        no_odom = not self._main.settings.odometry.equipped()
        if no_gps and no_odom:
            q_error_named(
                self._main,
                self.NAME,
                "Absolute position connot be observed. Please review the sensor settings.",
            )
            return False

        return True

    @overrides
    def parameter_dict(self) -> dict:
        gps = self._main.settings.gps

        res = dict()
        res["state_estimator_eskf"] = {
            "use_barometer": False,  # TODO: 選択できるように
            "use_gps": gps.equipped(),
            "gps_horizontal_position_stddev_threshold": self.gps_hor_pos_stddev_threshold.get(),
            "gps_vertical_position_stddev_threshold": self.gps_ver_pos_stddev_threshold.get(),
            "geomag_observe_method": "yaw_only",
            "rotation_variance_grav": self._rot_var_grav.get(),
            "rotation_variance_geomag": self._rot_var_geomag.get(),
            "acc_bias_noise_var_log10": self._acc_bias_noise_var_log10.get(),
            "gyro_bias_noise_var_log10": self._gyro_bias_noise_var_log10.get(),
        }

        return res
