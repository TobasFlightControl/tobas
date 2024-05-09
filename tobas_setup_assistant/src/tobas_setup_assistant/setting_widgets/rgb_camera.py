from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..common import *


class RgbCameraWidget(BaseSettingWidget):
    NAME = "RGB Camera"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define RGB Camera"
        abst_text = "Configure the RGB camera settings. Please refer to the datasheet and enter the respective values."
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox("RGB Camera Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(False)
        self._rows.addWidget(self._equipped)

        link_description = "The name of the link to which the camera is attached."
        self.link = ParamGetterWidget_ComboBox("Link name", link_description)
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
            "Horizontal Field of View", fov_description, decimals=6, minimum=0.0, default=1.59174, suffix=" rad"
        )
        self._rows.addWidget(self.fov)

        image_width_description = ""
        self.image_width = ParamGetterWidget_SpinBox(
            "Image Width", image_width_description, minimum=1, default=848, suffix=" px"
        )
        self._rows.addWidget(self.image_width)

        image_height_description = ""
        self.image_height = ParamGetterWidget_SpinBox(
            "Image Height", image_height_description, minimum=1, default=480, suffix=" px"
        )
        self._rows.addWidget(self.image_height)

        depth_range_description = (
            "The range of depth observable by the camera. "
            "In the simulation, objects outside this range will be truncated."
        )
        self.depth_range = ParamGetterWidget_DoubleRange(
            "Depth Range", depth_range_description, minimum=0.0, default=(0.01, 500.0), suffix=" m"
        )
        self._rows.addWidget(self.depth_range)

        noise_stddev_description = ""
        self.noise_stddev = ParamGetterWidget_DoubleSpinBox(
            "Noise Standard Deviation", noise_stddev_description, decimals=6, minimum=0.0, default=0.007
        )
        self._rows.addWidget(self.noise_stddev)

        self._rows.addStretch()
        self._update_visibility()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._equipped.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

    @override
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
            self.image_width.setVisible(True)
            self.image_height.setVisible(True)
            self.depth_range.setVisible(True)
            self.noise_stddev.setVisible(True)
        else:
            self.link.setVisible(False)
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.fov.setVisible(False)
            self.image_width.setVisible(False)
            self.image_height.setVisible(False)
            self.depth_range.setVisible(False)
            self.noise_stddev.setVisible(False)

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        self.link.set_choices(self._main.urdf_parser.link_names_available_in_gazebo())
