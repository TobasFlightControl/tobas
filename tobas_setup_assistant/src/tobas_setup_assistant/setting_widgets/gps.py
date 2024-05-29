from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override

from ..common import SENSOR_OFFSET_DESCRIPTION
from ..parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_DoubleSpinBox, ParamGetterWidget_Vector3d
from .base_setting import OptionalDeviceWidget


class GpsWidget(OptionalDeviceWidget):
    NAME = "GPS"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Global Positioning System"
        abst_text = ""  # TODO
        super().__init__(main, title_text, abst_text, True)

        self.offset = ParamGetterWidget_Vector3d("Offset", SENSOR_OFFSET_DESCRIPTION, suffix=" m")
        self._add_config_widget(self.offset)

        update_rate_description = ""  # TODO
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate", update_rate_description, minimum=1, default=5, suffix=" Hz"
        )
        self._add_config_widget(self.update_rate)

        delay_description = ""  # TODO
        self.delay = ParamGetterWidget_DoubleSpinBox(
            "Communication delay", delay_description, decimals=2, minimum=0.0, default=0.2, suffix=" s"
        )
        self._add_config_widget(self.delay)

        pos_corr_time_description = ""  # TODO
        self.pos_corr_time = ParamGetterWidget_SpinBox(
            "Position correction time constant", pos_corr_time_description, minimum=1, default=10, suffix=" s"
        )
        self._add_config_widget(self.pos_corr_time)

        horizontal_pos_accuracy_description = ""  # TODO
        self.horizontal_pos_accuracy = ParamGetterWidget_DoubleSpinBox(
            "Horizontal position accuracy",
            horizontal_pos_accuracy_description,
            decimals=2,
            minimum=0.0,
            default=2.0,
            suffix=" m",
        )
        self._add_config_widget(self.horizontal_pos_accuracy)

        vertical_pos_accuracy_description = ""  # TODO
        self.vertical_pos_accuracy = ParamGetterWidget_DoubleSpinBox(
            "Hertical position accuracy",
            vertical_pos_accuracy_description,
            decimals=2,
            minimum=0.0,
            default=4.0,
            suffix=" m",
        )
        self._add_config_widget(self.vertical_pos_accuracy)

        horizontal_vel_stddev_description = ""  # TODO
        self.horizontal_vel_stddev = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal speed noise",
            horizontal_vel_stddev_description,
            decimals=2,
            minimum=0.0,
            default=0.1,
            suffix=" m/s",
        )
        self._add_config_widget(self.horizontal_vel_stddev)

        vertical_vel_stddev_description = ""  # TODO
        self.vertical_vel_stddev = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical speed noise",
            vertical_vel_stddev_description,
            decimals=2,
            minimum=0.0,
            default=0.1,
            suffix=" m/s",
        )
        self._add_config_widget(self.vertical_vel_stddev)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        if not self.equipped():
            return True

        return True
