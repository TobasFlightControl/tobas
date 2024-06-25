from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override

from tobas_std_tools_py.enum import ExtEnum
from tobas_rqt_tools.messages import q_error_named

from ..common import CAMERA_LINK_DESCRIPTION, CAMERA_OFFSET_DESCRIPTION
from ..parameter_getters import (
    ParamGetterWidget_SpinBox,
    ParamGetterWidget_DoubleSpinBox,
    ParamGetterWidget_ComboBox,
    ParamGetterWidget_Pose,
    ParamGetterWidget_DoubleRange,
)
from .base_setting import OptionalDeviceWidget


class DepthNoiseModel(ExtEnum):
    KINECT = "Kinect"
    PMD = "PMD"
    D435 = "D435"


class DepthCameraWidget(OptionalDeviceWidget):
    NAME = "Depth Camera"
    TITLE_TEXT = "Define Depth Camera"
    ABST_TEXT = "Configure the depth camera settings. Please refer to the datasheet and input the respective values."

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main, False)

        self.link = ParamGetterWidget_ComboBox("Link Name", CAMERA_LINK_DESCRIPTION)
        self._add_param_widget(self.link)

        self.offset = ParamGetterWidget_Pose("Offset", CAMERA_OFFSET_DESCRIPTION)
        self._add_param_widget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update Rate", update_rate_description, minimum=1, default=30, suffix=" Hz"
        )
        self._add_param_widget(self.update_rate)

        fov_description = ""
        self.fov = ParamGetterWidget_DoubleSpinBox(
            "Horizontal Field of View", fov_description, decimals=6, minimum=0.0, default=1.59174, suffix=" rad"
        )
        self._add_param_widget(self.fov)

        baseline_description = ""
        self.baseline = ParamGetterWidget_DoubleSpinBox(
            "Baseline", baseline_description, decimals=6, minimum=0.0, default=0.05, suffix=""  # TODO  # TODO
        )
        self._add_param_widget(self.baseline)

        image_width_description = ""
        self.image_width = ParamGetterWidget_SpinBox(
            "Image Width", image_width_description, minimum=1, default=848, suffix=" px"
        )
        self._add_param_widget(self.image_width)

        image_height_description = ""
        self.image_height = ParamGetterWidget_SpinBox(
            "Image Height", image_height_description, minimum=1, default=480, suffix=" px"
        )
        self._add_param_widget(self.image_height)

        depth_range_description = (
            "The range of depth observable by the camera. "
            "In the simulation, objects outside this range will be truncated."
        )
        self.depth_range = ParamGetterWidget_DoubleRange(
            "Depth Range", depth_range_description, minimum=0.0, default=(0.195, 50.0), suffix=" m"
        )
        self._add_param_widget(self.depth_range)

        noise_model_description = ""
        self.noise_model = ParamGetterWidget_ComboBox(
            "Depth Noise Model", noise_model_description, DepthNoiseModel.values()
        )
        self._add_param_widget(self.noise_model)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        self.link.set_choices(self._main.urdf_parser.link_names_available_in_gazebo())

    @override
    def is_valid(self) -> bool:
        if not self.equipped():
            return True

        if not self.depth_range.is_valid():
            q_error_named(self._main, self.NAME, "Invalid depth range.")
            return False

        return True
