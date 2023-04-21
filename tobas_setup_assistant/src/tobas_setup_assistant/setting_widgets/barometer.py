from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..constants import *
from ..parameter_getters import *
from ..utils import add_expanding_widget


class BarometerWidget(BaseSettingWidget):

    NAME = "Barometer"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Air Pressure Sensor'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        update_rate_description = "TODO: instruction"
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=100,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        pressure_var_description = "TODO: instruction"
        self.pressure_var = ParamGetterWidget_DoubleSpinBox(
            "the air pressure variance",
            pressure_var_description,
            decimals=2,
            minimum=0.,
            default=1.,
            suffix=" Pa^2"
        )
        self._rows.addWidget(self.pressure_var)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        return True

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.link.box.addItems(body_choices)
