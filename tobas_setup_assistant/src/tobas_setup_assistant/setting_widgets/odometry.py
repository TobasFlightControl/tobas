from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from .base_setting import BaseSettingWidget
from ..constants import *
from ..parameter_getters import *


class OdometryWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Odometry Sensor"
        abst_text = "TODO: abstruct"
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with odometry sensor.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        update_rate_description = "TODO: instruction"
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=10,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        # TODO: Covariance Image getter

        pos_normal_noise_std_descripiton = "TODO: instruction"
        self.pos_normal_noise_std = ParamGetterWidget_Vector3d(
            "Position normal noise std. dev",
            pos_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.01] * 3,
            suffix=" m",
        )
        self._rows.addWidget(self.pos_normal_noise_std)

        rot_normal_noise_std_descripiton = "TODO: instruction"
        self.rot_normal_noise_std = ParamGetterWidget_Vector3d(
            "Rotation normal noise std. dev",
            rot_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.017] * 3,
            suffix=" rad",
        )
        self._rows.addWidget(self.rot_normal_noise_std)

        linvel_normal_noise_std_descripiton = "TODO: instruction"
        self.linvel_normal_noise_std = ParamGetterWidget_Vector3d(
            "Linear velocity normal noise std. dev",
            linvel_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" m/s",
        )
        self._rows.addWidget(self.linvel_normal_noise_std)

        angvel_normal_noise_std_descripiton = "TODO: instruction"
        self.angvel_normal_noise_std = ParamGetterWidget_Vector3d(
            "Angular velocity normal noise std. dev",
            angvel_normal_noise_std_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" rad/s",
        )
        self._rows.addWidget(self.angvel_normal_noise_std)

        pos_uniform_noise_scale_descripiton = "TODO: instruction"
        self.pos_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Position uniform noise scale",
            pos_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" m",
        )
        self._rows.addWidget(self.pos_uniform_noise_scale)

        rot_uniform_noise_scale_descripiton = "TODO: instruction"
        self.rot_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Rotation uniform noise scale",
            rot_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" rad",
        )
        self._rows.addWidget(self.rot_uniform_noise_scale)

        linvel_uniform_noise_scale_descripiton = "TODO: instruction"
        self.linvel_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Linear velocity uniform noise scale",
            linvel_uniform_noise_scale_descripiton,
            minimum=[0.] * 3,
            default=[0.] * 3,
            suffix=" m/s",
        )
        self._rows.addWidget(self.linvel_uniform_noise_scale)

        angvel_uniform_noise_scale_descripiton = "TODO: instruction"
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
        self.no_sensor.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        return True

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.link.setVisible(False)
            self.update_rate.setVisible(False)
            self.pos_normal_noise_std.setVisible(False)
            self.rot_normal_noise_std.setVisible(False)
            self.linvel_normal_noise_std.setVisible(False)
            self.angvel_normal_noise_std.setVisible(False)
            self.pos_uniform_noise_scale.setVisible(False)
            self.rot_uniform_noise_scale.setVisible(False)
            self.linvel_uniform_noise_scale.setVisible(False)
            self.angvel_uniform_noise_scale.setVisible(False)
        else:
            self.link.setVisible(True)
            self.update_rate.setVisible(True)
            self.pos_normal_noise_std.setVisible(True)
            self.rot_normal_noise_std.setVisible(True)
            self.linvel_normal_noise_std.setVisible(True)
            self.angvel_normal_noise_std.setVisible(True)
            self.pos_uniform_noise_scale.setVisible(True)
            self.rot_uniform_noise_scale.setVisible(True)
            self.linvel_uniform_noise_scale.setVisible(True)
            self.angvel_uniform_noise_scale.setVisible(True)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.nwu_fixed_link_names()
        self.link.box.addItems(body_choices)
