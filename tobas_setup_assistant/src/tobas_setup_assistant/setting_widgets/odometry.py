from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override

from ..common import SENSOR_OFFSET_DESCRIPTION
from ..parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_Vector3d
from .base_setting import OptionalDeviceWidget


class OdometryWidget(OptionalDeviceWidget):
    NAME = "Odometry"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Odometry Publisher"
        abst_text = (
            "Configure the settings for the device issuing odometry data. "
            "Please refer to the datasheet and enter the respective values. "
            "This includes devices like wheel encoders or Visual Inertial Odometry (VIO)."
        )
        super().__init__(main, title_text, abst_text, False)

        self.offset = ParamGetterWidget_Vector3d("Offset", SENSOR_OFFSET_DESCRIPTION, suffix=" m")
        self._add_config_widget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate", update_rate_description, minimum=1, default=10, suffix=" Hz"
        )
        self._add_config_widget(self.update_rate)

        # TODO: Covariance Image getter

        pos_normal_noise_std_descripiton = ""
        self.pos_normal_noise_std = ParamGetterWidget_Vector3d(
            "Position normal noise std. dev",
            pos_normal_noise_std_descripiton,
            minimum=[0.0] * 3,
            default=[0.01] * 3,
            suffix=" m",
        )
        self._add_config_widget(self.pos_normal_noise_std)

        rot_normal_noise_std_descripiton = ""
        self.rot_normal_noise_std = ParamGetterWidget_Vector3d(
            "Rotation normal noise std. dev",
            rot_normal_noise_std_descripiton,
            minimum=[0.0] * 3,
            default=[0.017] * 3,
            suffix=" rad",
        )
        self._add_config_widget(self.rot_normal_noise_std)

        linvel_normal_noise_std_descripiton = ""
        self.linvel_normal_noise_std = ParamGetterWidget_Vector3d(
            "Linear velocity normal noise std. dev",
            linvel_normal_noise_std_descripiton,
            minimum=[0.0] * 3,
            default=[0.0] * 3,
            suffix=" m/s",
        )
        self._add_config_widget(self.linvel_normal_noise_std)

        angvel_normal_noise_std_descripiton = ""
        self.angvel_normal_noise_std = ParamGetterWidget_Vector3d(
            "Angular velocity normal noise std. dev",
            angvel_normal_noise_std_descripiton,
            minimum=[0.0] * 3,
            default=[0.0] * 3,
            suffix=" rad/s",
        )
        self._add_config_widget(self.angvel_normal_noise_std)

        pos_uniform_noise_scale_descripiton = ""
        self.pos_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Position uniform noise scale",
            pos_uniform_noise_scale_descripiton,
            minimum=[0.0] * 3,
            default=[0.0] * 3,
            suffix=" m",
        )
        self._add_config_widget(self.pos_uniform_noise_scale)

        rot_uniform_noise_scale_descripiton = ""
        self.rot_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Rotation uniform noise scale",
            rot_uniform_noise_scale_descripiton,
            minimum=[0.0] * 3,
            default=[0.0] * 3,
            suffix=" rad",
        )
        self._add_config_widget(self.rot_uniform_noise_scale)

        linvel_uniform_noise_scale_descripiton = ""
        self.linvel_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Linear velocity uniform noise scale",
            linvel_uniform_noise_scale_descripiton,
            minimum=[0.0] * 3,
            default=[0.0] * 3,
            suffix=" m/s",
        )
        self._add_config_widget(self.linvel_uniform_noise_scale)

        angvel_uniform_noise_scale_descripiton = ""
        self.angvel_uniform_noise_scale = ParamGetterWidget_Vector3d(
            "Angular velocity uniform noise scale",
            angvel_uniform_noise_scale_descripiton,
            minimum=[0.0] * 3,
            default=[0.0] * 3,
            suffix=" rad/s",
        )
        self._add_config_widget(self.angvel_uniform_noise_scale)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        if not self.equipped():
            return True

        return True

    @override
    def dump_settings(self) -> dict:
        raise NotImplementedError()  # TODO

    @override
    def load_settings(self, data: dict) -> None:
        raise NotImplementedError()  # TODO
