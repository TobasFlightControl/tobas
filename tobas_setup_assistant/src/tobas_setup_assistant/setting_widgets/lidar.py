from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..common import *


class LidarWidget(BaseSettingWidget):

    NAME = "LiDAR"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define LiDAR"
        abst_text = "3D LiDARの設定を行います．データシートを確認し，各値を入力してください．"
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox("LiDAR Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(False)
        self._rows.addWidget(self._equipped)

        offset_description = "ルートリンクに対するLiDARの位置のオフセット．"
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

        hor_samples_description = ""
        self.hor_samples = ParamGetterWidget_SpinBox(
            "The number of horizontal samples",
            hor_samples_description,
            minimum=1,
            default=100,
        )
        self._rows.addWidget(self.hor_samples)

        ver_samples_description = ""
        self.ver_samples = ParamGetterWidget_SpinBox(
            "The number of vertical samples",
            ver_samples_description,
            minimum=1,
            default=360,
        )
        self._rows.addWidget(self.ver_samples)

        hor_fov_description = ""
        self.hor_fov = ParamGetterWidget_DoubleRange(
            "Horizontal Field of View",
            hor_fov_description,
            decimals=3,
            default=(0., 2 * math.pi),
            suffix=" rad",
        )
        self._rows.addWidget(self.hor_fov)

        ver_fov_description = ""
        self.ver_fov = ParamGetterWidget_DoubleRange(
            "Vertical Field of View",
            ver_fov_description,
            decimals=3,
            default=(math.radians(-7.22), math.radians(55.22)),
            suffix=" rad",
        )
        self._rows.addWidget(self.ver_fov)

        range_description = ""
        self.range = ParamGetterWidget_DoubleRange(
            "Laser distance range",
            range_description,
            decimals=3,
            default=(0.1, 200.),
            suffix=" m",
        )
        self._rows.addWidget(self.range)

        resolution_description = ""
        self.resolution = ParamGetterWidget_DoubleSpinBox(
            "Distance resolution",
            resolution_description,
            decimals=3,
            minimum=1e-3,
            default=2e-3,
            suffix=" m",
        )
        self._rows.addWidget(self.resolution)

        noise_stddev_description = ""
        self.noise_stddev = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation of gaussian noise",
            noise_stddev_description,
            decimals=3,
            minimum=0.,
            default=0.01,
            suffix=" m",
        )
        self._rows.addWidget(self.noise_stddev)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self._equipped.toggled.connect(self._update_visibility)

    def is_valid(self) -> bool:
        if not self.hor_fov.is_valid():
            q_error_named(self._main, self.NAME, "Horizontal Field of View is invalid.")
            return False
        if not self.ver_fov.is_valid():
            q_error_named(self._main, self.NAME, "Vertical Field of View is invalid.")
            return False
        if not self.range.is_valid():
            q_error_named(self._main, self.NAME, "Laser distance range is invalid.")
            return False

        return True

    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self._equipped.isChecked():
            self.offset.setVisible(True)
            self.update_rate.setVisible(True)
            self.hor_samples.setVisible(True)
            self.hor_fov.setVisible(True)
            self.ver_samples.setVisible(True)
            self.ver_fov.setVisible(True)
            self.range.setVisible(True)
            self.resolution.setVisible(True)
            self.noise_stddev.setVisible(True)
        else:
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.hor_samples.setVisible(False)
            self.hor_fov.setVisible(False)
            self.ver_samples.setVisible(False)
            self.ver_fov.setVisible(False)
            self.range.setVisible(False)
            self.resolution.setVisible(False)
            self.noise_stddev.setVisible(False)
