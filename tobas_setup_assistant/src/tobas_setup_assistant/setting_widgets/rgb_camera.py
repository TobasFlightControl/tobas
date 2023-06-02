from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *


class RgbCameraWidget(BaseSettingWidget):

    NAME = "RGB Camera"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define RGB Camera"
        abst_text = "RGBカメラの設定を行います．データシートを確認し，各値を入力してください．"
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with RGB camera.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        link_description = "カメラが取り付けられたフレームの名前．"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        offset_description = "選択したリンクに対するカメラ位置のオフセット．"
        self.offset = ParamGetterWidget_Pose(
            "Offset",
            offset_description,
        )
        self._rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update Rate",
            update_rate_description,
            minimum=1,
            default=30,
            suffix=" Hz"
        )
        self._rows.addWidget(self.update_rate)

        fov_description = ""
        self.fov = ParamGetterWidget_DoubleSpinBox(
            "Horizontal Field of View",
            fov_description,
            decimals=6,
            minimum=0.,
            default=1.59174,
            suffix=" rad",
        )
        self._rows.addWidget(self.fov)

        image_width_description = ""
        self.image_width = ParamGetterWidget_SpinBox(
            "Image Width",
            image_width_description,
            minimum=1,
            default=848,
            suffix=" px",
        )
        self._rows.addWidget(self.image_width)

        image_height_description = ""
        self.image_height = ParamGetterWidget_SpinBox(
            "Image Height",
            image_height_description,
            minimum=1,
            default=480,
            suffix=" px",
        )
        self._rows.addWidget(self.image_height)

        depth_range_description = "カメラで観測可能な深さの範囲．"\
            + "シミュレーションでは，この範囲外にある物体は切り捨てられます．"
        self.depth_range = ParamGetterWidget_DoubleRange(
            "Depth Range",
            depth_range_description,
            minimum=0.,
            default=(0.01, 500.),
            suffix=" m",
        )
        self._rows.addWidget(self.depth_range)

        noise_stddev_description = ""
        self.noise_stddev = ParamGetterWidget_DoubleSpinBox(
            "Noise Standard Deviation",
            noise_stddev_description,
            decimals=6,
            minimum=0.,
            default=0.007,
        )
        self._rows.addWidget(self.noise_stddev)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        if self.no_sensor.isChecked():
            return True

        if not self.depth_range.is_valid():
            q_error_named(self._main, self.NAME, "Invalid depth range.")
            return False

        return True

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.link.setVisible(False)
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.fov.setVisible(False)
            self.image_width.setVisible(False)
            self.image_height.setVisible(False)
            self.depth_range.setVisible(False)
            self.noise_stddev.setVisible(False)
        else:
            self.link.setVisible(True)
            self.offset.setVisible(True)
            self.update_rate.setVisible(True)
            self.fov.setVisible(True)
            self.image_width.setVisible(True)
            self.image_height.setVisible(True)
            self.depth_range.setVisible(True)
            self.noise_stddev.setVisible(True)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.link_names()
        self.link.box.addItems(body_choices)
