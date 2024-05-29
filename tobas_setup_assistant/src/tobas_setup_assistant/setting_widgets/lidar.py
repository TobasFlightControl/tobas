from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import math
from overrides import override

from tobas_rqt_tools.messages import q_error_named

from ..common import SENSOR_OFFSET_DESCRIPTION
from ..parameter_getters import (
    ParamGetterWidget_SpinBox,
    ParamGetterWidget_Vector3d,
    ParamGetterWidget_DoubleSpinBox,
    ParamGetterWidget_DoubleRange,
)
from .base_setting import OptionalDeviceWidget


class LidarWidget(OptionalDeviceWidget):
    NAME = "LiDAR"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define LiDAR"
        abst_text = "Configure the 3D LiDAR settings. " "Please refer to the datasheet and input the respective values."
        super().__init__(main, title_text, abst_text, False)

        self.offset = ParamGetterWidget_Vector3d("Offset", SENSOR_OFFSET_DESCRIPTION, suffix=" m")
        self._add_config_widget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate", update_rate_description, minimum=1, default=10, suffix=" Hz"
        )
        self._add_config_widget(self.update_rate)

        hor_samples_description = ""
        self.hor_samples = ParamGetterWidget_SpinBox(
            "The number of horizontal samples", hor_samples_description, minimum=1, default=100
        )
        self._add_config_widget(self.hor_samples)

        ver_samples_description = ""
        self.ver_samples = ParamGetterWidget_SpinBox(
            "The number of vertical samples", ver_samples_description, minimum=1, default=360
        )
        self._add_config_widget(self.ver_samples)

        hor_fov_description = ""
        self.hor_fov = ParamGetterWidget_DoubleRange(
            "Horizontal Field of View", hor_fov_description, decimals=3, default=(0.0, 2 * math.pi), suffix=" rad"
        )
        self._add_config_widget(self.hor_fov)

        ver_fov_description = ""
        self.ver_fov = ParamGetterWidget_DoubleRange(
            "Vertical Field of View",
            ver_fov_description,
            decimals=3,
            default=(math.radians(-7.22), math.radians(55.22)),
            suffix=" rad",
        )
        self._add_config_widget(self.ver_fov)

        range_description = ""
        self.range = ParamGetterWidget_DoubleRange(
            "Laser distance range", range_description, decimals=3, default=(0.1, 200.0), suffix=" m"
        )
        self._add_config_widget(self.range)

        resolution_description = ""
        self.resolution = ParamGetterWidget_DoubleSpinBox(
            "Distance resolution", resolution_description, decimals=3, minimum=1e-3, default=2e-3, suffix=" m"
        )
        self._add_config_widget(self.resolution)

        noise_stddev_description = ""
        self.noise_stddev = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation of gaussian noise",
            noise_stddev_description,
            decimals=3,
            minimum=0.0,
            default=0.01,
            suffix=" m",
        )
        self._add_config_widget(self.noise_stddev)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        if not self.equipped():
            return True

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
