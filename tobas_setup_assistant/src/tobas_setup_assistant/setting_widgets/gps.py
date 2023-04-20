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


class GpsWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Global Positioning System'
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
            default=5,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        horizontal_pos_std_description = "TODO: instruction"
        self.horizontal_pos_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal position noise",
            horizontal_pos_std_description,
            decimals=2,
            minimum=0.,
            default=3.,
            suffix=" m",
        )
        self._rows.addWidget(self.horizontal_pos_std)

        vertical_pos_std_description = "TODO: instruction"
        self.vertical_pos_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical position noise",
            vertical_pos_std_description,
            decimals=2,
            minimum=0.,
            default=6.,
            suffix=" m",
        )
        self._rows.addWidget(self.vertical_pos_std)

        horizontal_vel_std_description = "TODO: instruction"
        self.horizontal_vel_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal speed noise",
            horizontal_vel_std_description,
            decimals=2,
            minimum=0.,
            default=0.1,
            suffix=" m/s",
        )
        self._rows.addWidget(self.horizontal_vel_std)

        vertical_vel_std_description = "TODO: instruction"
        self.vertical_vel_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical speed noise",
            vertical_vel_std_description,
            decimals=2,
            minimum=0.,
            default=0.1,
            suffix=" m/s",
        )
        self._rows.addWidget(self.vertical_vel_std)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.link.box.addItems(body_choices)
