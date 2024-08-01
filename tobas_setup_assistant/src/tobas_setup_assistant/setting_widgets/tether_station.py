from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override

from ..parameter_getters import (
    ParamGetterWidget_ComboBox,
    ParamGetterWidget_Vector3d,
    ParamGetterWidget_SpinBox,
    ParamGetterWidget_DoubleSpinBox,
)
from .base_setting import OptionalDeviceWidget


class TetherStationWidget(OptionalDeviceWidget):
    NAME = "Tether Station"
    TITLE_TEXT = "Define Tether Station"
    ABST_TEXT = (
        "A Tether Station is a device that connects a drone to the ground with a constantly tensioned cable. "
        "It can prevent the drone from straying by using an emergency brake and "
        "extend flight duration by providing power through the cable."
    )

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main, False)

        link_description = "The name of the link to which the end of the cable is attached."
        self.link = ParamGetterWidget_ComboBox("Link Name", link_description)
        self._add_param_widget(self.link)

        drone_end_description = "The connection point between the cable and the drone wrt. the selected link frame."
        self.drone_end = ParamGetterWidget_Vector3d("Connection Point (Drone)", drone_end_description)
        self._add_param_widget(self.drone_end)

        world_end_description = "The connection point between the cable and the ground wrt. the world frame."
        self.world_end = ParamGetterWidget_Vector3d("Connection Point (Ground)", world_end_description)
        self._add_param_widget(self.world_end)

        tension_description = "The constant tension of the cable. It can be modified during simulation."
        self.tension = ParamGetterWidget_DoubleSpinBox(
            "Tension",
            tension_description,
            decimals=2,
            minimum=0.0,
            default=1.0,
            suffix=" N",
        )
        self._add_param_widget(self.tension)

        max_length_description = "The maximum length of the cable. It can be modified during simulation."
        self.max_length = ParamGetterWidget_DoubleSpinBox(
            "Maximum Length",
            max_length_description,
            decimals=1,
            minimum=1.0,
            default=5.0,
            suffix=" m",
        )
        self._add_param_widget(self.max_length)

        young_modulus_description = "The young modulus of the cable."
        self.young_modulus = ParamGetterWidget_SpinBox(
            "Young Modulus",
            young_modulus_description,
            minimum=1,
            default=200,
            suffix=" MPa",
        )
        self._add_param_widget(self.young_modulus)

        cross_sectional_area_description = "The cross-sectional area of the cable."
        self.cross_sectional_area = ParamGetterWidget_DoubleSpinBox(
            "Cross-Sectional Area",
            cross_sectional_area_description,
            decimals=1,
            minimum=0.1,
            default=1.0,
            suffix=" mm^2",
        )
        self._add_param_widget(self.cross_sectional_area)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        self.link.set_choices(self._main.urdf_parser.link_names_available_in_gazebo())

    @override
    def is_valid(self) -> bool:
        if not self.equipped():
            return True

        return True
