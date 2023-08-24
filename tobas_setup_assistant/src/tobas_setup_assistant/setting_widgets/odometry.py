from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from .base_setting import BaseSettingWidget
from ..common import *
from ..parameter_getters import *


class OdometryWidget(BaseSettingWidget):

    NAME = "Odometry"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Odometry Publisher"
        abst_text = "オドメトリを発行する機器の設定を行います．データシートを確認し，各値を入力してください．"\
            + "ホイールエンコーダや，VIO (Visual Inertial Odometry) などが該当します．"
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox("Odometry Publisher Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(False)
        self._rows.addWidget(self._equipped)

        offset_description = "ルートリンクに対するオドメトリを得るフレームのオフセット．"
        self.offset = ParamGetterWidget_Vector3d(
            "Offset",
            offset_description,
            suffix=" m",
        )
        self._rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=10,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        # TODO: Covariance Image getter

        pos_normal_noise_std_descripiton = ""
        self.pos_normal_noise_std = ParamGetterWidget_Vector3d(
            "Position normal noise std. dev",
            pos_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.01] * 3,
            suffix=" m",
        )
        self._rows.addWidget(self.pos_normal_noise_std)

        rot_normal_noise_std_descripiton = ""
        self.rot_normal_noise_std = ParamGetterWidget_Vector3d(
            "Rotation normal noise std. dev",
            rot_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.017] * 3,
            suffix=" rad",
        )
        self._rows.addWidget(self.rot_normal_noise_std)

        linvel_normal_noise_std_descripiton = ""
        self.linvel_normal_noise_std = ParamGetterWidget_Vector3d(
            "Linear velocity normal noise std. dev",
            linvel_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" m/s",
        )
        self._rows.addWidget(self.linvel_normal_noise_std)

        angvel_normal_noise_std_descripiton = ""
        self.angvel_normal_noise_std = ParamGetterWidget_Vector3d(
            "Angular velocity normal noise std. dev",
            angvel_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" rad/s",
        )
        self._rows.addWidget(self.angvel_normal_noise_std)

        pos_uniform_noise_scale_descripiton = ""
        self.pos_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Position uniform noise scale",
            pos_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" m",
        )
        self._rows.addWidget(self.pos_uniform_noise_scale)

        rot_uniform_noise_scale_descripiton = ""
        self.rot_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Rotation uniform noise scale",
            rot_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" rad",
        )
        self._rows.addWidget(self.rot_uniform_noise_scale)

        linvel_uniform_noise_scale_descripiton = ""
        self.linvel_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Linear velocity uniform noise scale",
            linvel_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" m/s",
        )
        self._rows.addWidget(self.linvel_uniform_noise_scale)

        angvel_uniform_noise_scale_descripiton = ""
        self.angvel_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Angular velocity uniform noise scale",
            angvel_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" rad/s",
        )
        self._rows.addWidget(self.angvel_uniform_noise_scale)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self._equipped.toggled.connect(self._update_visibility)

    def is_valid(self) -> bool:
        if not self._equipped.isChecked():
            return True

        return True

    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self._equipped.isChecked():
            self.offset.setVisible(True)
            self.update_rate.setVisible(True)
            self.pos_normal_noise_std.setVisible(True)
            self.rot_normal_noise_std.setVisible(True)
            self.linvel_normal_noise_std.setVisible(True)
            self.angvel_normal_noise_std.setVisible(True)
            self.pos_uniform_noise_scale.setVisible(True)
            self.rot_uniform_noise_scale.setVisible(True)
            self.linvel_uniform_noise_scale.setVisible(True)
            self.angvel_uniform_noise_scale.setVisible(True)
        else:
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.pos_normal_noise_std.setVisible(False)
            self.rot_normal_noise_std.setVisible(False)
            self.linvel_normal_noise_std.setVisible(False)
            self.angvel_normal_noise_std.setVisible(False)
            self.pos_uniform_noise_scale.setVisible(False)
            self.rot_uniform_noise_scale.setVisible(False)
            self.linvel_uniform_noise_scale.setVisible(False)
            self.angvel_uniform_noise_scale.setVisible(False)
