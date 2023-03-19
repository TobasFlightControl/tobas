from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..const import *
from ..parameter_getters import *


class GpsWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Global Positioning System'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        self.use_custom_gps = QCheckBox("Use custom GPS")
        self.use_custom_gps.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.use_custom_gps)

        update_rate_description = "TODO: instruction"
        self.update_rate = ParamGetterWidget_DoubleSpinBox(
            "Update rate [Hz]",
            update_rate_description,
            minimum=0.,
            default=5.,
        )
        self._rows.addWidget(self.update_rate)

        horizontal_pos_std_description = "TODO: instruction"
        self.horizontal_pos_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal position noise [m]",
            horizontal_pos_std_description,
            minimum=0.,
            default=3.,
        )
        self._rows.addWidget(self.horizontal_pos_std)

        vertical_pos_std_description = "TODO: instruction"
        self.vertical_pos_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical position noise [m]",
            vertical_pos_std_description,
            minimum=0.,
            default=6.,
        )
        self._rows.addWidget(self.vertical_pos_std)

        horizontal_vel_std_description = "TODO: instruction"
        self.horizontal_vel_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal speed noise [m/s]",
            horizontal_vel_std_description,
            minimum=0.,
            default=0.1,
        )
        self._rows.addWidget(self.horizontal_vel_std)

        vertical_vel_std_description = "TODO: instruction"
        self.vertical_vel_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical speed noise [m/s]",
            vertical_vel_std_description,
            minimum=0.,
            default=0.1,
        )
        self._rows.addWidget(self.vertical_vel_std)

        self._add_dummy_widget()
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.use_custom_gps.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.link.box.addItems(body_choices)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.use_custom_gps.isChecked():
            self.update_rate.setVisible(True)
            self.horizontal_pos_std.setVisible(True)
            self.vertical_pos_std.setVisible(True)
            self.horizontal_vel_std.setVisible(True)
            self.vertical_vel_std.setVisible(True)
        else:
            self.update_rate.setVisible(False)
            self.horizontal_pos_std.setVisible(False)
            self.vertical_pos_std.setVisible(False)
            self.horizontal_vel_std.setVisible(False)
            self.vertical_vel_std.setVisible(False)
