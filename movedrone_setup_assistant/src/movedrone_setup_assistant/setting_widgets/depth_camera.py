from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *


class DepthCameraWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Depth Camera'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with depth camera.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        offset_description = "TODO: instruction"
        self.offset = ParamGetterWidget_Pose(
            "Offset",
            offset_description,
        )
        self._rows.addWidget(self.offset)

        update_rate_description = "TODO: instruction"
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update Rate",
            update_rate_description,
            minimum=1,
            default=30,
            suffix=" Hz"
        )
        self._rows.addWidget(self.update_rate)

        fov_description = "TODO: instruction"
        self.fov = ParamGetterWidget_DoubleSpinBox(
            "Horizontal Field of View",
            fov_description,
            decimals=6,
            minimum=0.,
            default=1.59174,
            suffix=" rad",
        )
        self._rows.addWidget(self.fov)

        baseline_description = "TODO: instruction"
        self.baseline = ParamGetterWidget_DoubleSpinBox(
            "Baseline",  # TODO
            baseline_description,
            decimals=6,
            minimum=0.,
            default=0.05,
            suffix="",  # TODO
        )
        self._rows.addWidget(self.baseline)

        image_width_description = "TODO: instruction"
        self.image_width = ParamGetterWidget_SpinBox(
            "Image Width",
            image_width_description,
            minimum=1,
            default=848,
            suffix=" px",
        )
        self._rows.addWidget(self.image_width)

        image_height_description = "TODO: instruction"
        self.image_height = ParamGetterWidget_SpinBox(
            "Image Height",
            image_height_description,
            minimum=1,
            default=480,
            suffix=" px",
        )
        self._rows.addWidget(self.image_height)

        depth_range_description = "TODO: instruction"
        self.depth_range = ParamGetterWidget_DoubleRange(
            "Depth Range",
            depth_range_description,
            minimum=0.,
            default=(0.195, 50.),
            suffix=" m",
        )
        self._rows.addWidget(self.depth_range)

        noise_model_description = "TODO: instruction"
        self.noise_model = ParamGetterWidget_ComboBox(
            "Depth Noise Model",
            noise_model_description,
            ["Kinect", "PMD", "D435"],
        )
        self._rows.addWidget(self.noise_model)

        self._add_dummy_widget()
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.link.setVisible(False)
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.fov.setVisible(False)
            self.baseline.setVisible(False)
            self.image_width.setVisible(False)
            self.image_height.setVisible(False)
            self.depth_range.setVisible(False)
            self.noise_model.setVisible(False)
        else:
            self.link.setVisible(True)
            self.offset.setVisible(True)
            self.update_rate.setVisible(True)
            self.fov.setVisible(True)
            self.baseline.setVisible(True)
            self.image_width.setVisible(True)
            self.image_height.setVisible(True)
            self.depth_range.setVisible(True)
            self.noise_model.setVisible(True)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.link.box.addItems(body_choices)
