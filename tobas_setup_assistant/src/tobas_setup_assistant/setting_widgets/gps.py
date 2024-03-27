from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..common import *
from ..parameter_getters import *


class GpsWidget(BaseSettingWidget):
    NAME = "GPS"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Global Positioning System"
        abst_text = ""
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox("GPS Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(True)
        self._rows.addWidget(self._equipped)

        self.offset = ParamGetterWidget_Vector3d(
            "Offset",
            SENSOR_OFFSET_DESCRIPTION,
            suffix=" m",
        )
        self._rows.addWidget(self.offset)

        update_rate_description = ""  # TODO
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=5,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        delay_description = ""  # TODO
        self.delay = ParamGetterWidget_DoubleSpinBox(
            "Communication delay",
            delay_description,
            decimals=2,
            minimum=0.0,
            default=0.2,
            suffix=" s",
        )
        self._rows.addWidget(self.delay)

        pos_corr_time_description = ""  # TODO
        self.pos_corr_time = ParamGetterWidget_SpinBox(
            "Position correction time constant",
            pos_corr_time_description,
            minimum=1,
            default=10,
            suffix=" s",
        )
        self._rows.addWidget(self.pos_corr_time)

        horizontal_pos_accuracy_description = ""  # TODO
        self.horizontal_pos_accuracy = ParamGetterWidget_DoubleSpinBox(
            "Horizontal position accuracy",
            horizontal_pos_accuracy_description,
            decimals=2,
            minimum=0.0,
            default=2.0,
            suffix=" m",
        )
        self._rows.addWidget(self.horizontal_pos_accuracy)

        vertical_pos_accuracy_description = ""  # TODO
        self.vertical_pos_accuracy = ParamGetterWidget_DoubleSpinBox(
            "Hertical position accuracy",
            vertical_pos_accuracy_description,
            decimals=2,
            minimum=0.0,
            default=4.0,
            suffix=" m",
        )
        self._rows.addWidget(self.vertical_pos_accuracy)

        horizontal_vel_stddev_description = ""  # TODO
        self.horizontal_vel_stddev = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal speed noise",
            horizontal_vel_stddev_description,
            decimals=2,
            minimum=0.0,
            default=0.1,
            suffix=" m/s",
        )
        self._rows.addWidget(self.horizontal_vel_stddev)

        vertical_vel_stddev_description = ""  # TODO
        self.vertical_vel_stddev = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical speed noise",
            vertical_vel_stddev_description,
            decimals=2,
            minimum=0.0,
            default=0.1,
            suffix=" m/s",
        )
        self._rows.addWidget(self.vertical_vel_stddev)

        self._rows.addStretch()
        self._update_visibility()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._equipped.toggled.connect(self._update_visibility)

    @override
    def is_valid(self) -> bool:
        if not self._equipped.isChecked():
            return True

        return True

    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self._equipped.isChecked():
            self.offset.setVisible(True)
            self.update_rate.setVisible(True)
            self.horizontal_pos_accuracy.setVisible(True)
            self.vertical_pos_accuracy.setVisible(True)
            self.horizontal_vel_stddev.setVisible(True)
            self.vertical_vel_stddev.setVisible(True)
        else:
            self.offset.setVisible(False)
            self.update_rate.setVisible(False)
            self.horizontal_pos_accuracy.setVisible(False)
            self.vertical_pos_accuracy.setVisible(False)
            self.horizontal_vel_stddev.setVisible(False)
            self.vertical_vel_stddev.setVisible(False)
