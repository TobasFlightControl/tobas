from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from enum import Enum
from typing import List
from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..common import *


class DepthNoiseModel(Enum):
    KINECT = "Kinect"
    PMD = "PMD"
    D435 = "D435"

    @classmethod
    def get_all_values(cls) -> List[str]:
        return [item.value for item in cls]


class DepthCameraWidget(BaseSettingWidget):
    NAME = "Depth Camera"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Depth Camera"
        abst_text = "深度カメラの設定を行います．データシートを確認し，各値を入力してください．"
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox("Depth Camera Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(False)
        self._rows.addWidget(self._equipped)

        link_description = "カメラが取り付けられたフレームの名前．"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        self.offset = ParamGetterWidget_Pose("Offset", CAMERA_OFFSET_DESCRIPTION)
        self._rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update Rate", update_rate_description, minimum=1, default=30, suffix=" Hz"
        )
        self._rows.addWidget(self.update_rate)

        fov_description = ""
        self.fov = ParamGetterWidget_DoubleSpinBox(
            "Horizontal Field of View",
            fov_description,
            decimals=6,
            minimum=0.0,
            default=1.59174,
            suffix=" rad",
        )
        self._rows.addWidget(self.fov)

        baseline_description = ""
        self.baseline = ParamGetterWidget_DoubleSpinBox(
            "Baseline",  # TODO
            baseline_description,
            decimals=6,
            minimum=0.0,
            default=0.05,
            suffix="",  # TODO
        )
        self._rows.addWidget(self.baseline)

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

        depth_range_description = "カメラで観測可能な深さの範囲．" + "シミュレーションでは，この範囲外にある物体は切り捨てられます．"
        self.depth_range = ParamGetterWidget_DoubleRange(
            "Depth Range",
            depth_range_description,
            minimum=0.0,
            default=(0.195, 50.0),
            suffix=" m",
        )
        self._rows.addWidget(self.depth_range)

        noise_model_description = ""
        self.noise_model = ParamGetterWidget_ComboBox(
            "Depth Noise Model",
            noise_model_description,
            DepthNoiseModel.get_all_values(),
        )
        self._rows.addWidget(self.noise_model)

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._equipped.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_links)

    @overrides
    def is_valid(self) -> bool:
        if not self._equipped.isChecked():
            return True

        if not self.depth_range.is_valid():
            q_error_named(self._main, self.NAME, "Invalid depth range.")
            return False

        return True

    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self._equipped.isChecked():
            self.link.setVisible(True)
            self.offset.setVisible(True)
            self.update_rate.setVisible(True)
            self.fov.setVisible(True)
            self.baseline.setVisible(True)
            self.image_width.setVisible(True)
            self.image_height.setVisible(True)
            self.depth_range.setVisible(True)
            self.noise_model.setVisible(True)
        else:
            self.link.setVisible(False)
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.fov.setVisible(False)
            self.baseline.setVisible(False)
            self.image_width.setVisible(False)
            self.image_height.setVisible(False)
            self.depth_range.setVisible(False)
            self.noise_model.setVisible(False)

    @pyqtSlot()
    def _add_links(self) -> None:
        # Gazeboの仕様で，ルートリンクまたは可動関節をもつリンクのみ指定可能
        root_name = self._main.urdf_parser.get_root().name
        body_choices = [
            root_name
        ] + self._main.urdf_parser.link_names_with_mobile_joint()
        self.link.add_items(body_choices)
