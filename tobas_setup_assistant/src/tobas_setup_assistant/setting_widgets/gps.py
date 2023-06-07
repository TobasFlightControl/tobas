from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from .base_setting import BaseSettingWidget
from ..constants import *
from ..parameter_getters import *


class GpsWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Global Positioning System"
        abst_text = "GPSの設定を行います．データシートを確認し，各値を入力してください．"\
            + "緯度，経度，高度に加え，NWU世界座標系に対する絶対速度が得られるものを想定しています．"\
            + "Tobasのハードウェアを用いる場合は修正する必要はありません．"
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with GPS.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        link_description = "センサが取り付けられたフレームの名前．"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=5,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        horizontal_pos_std_description = ""
        self.horizontal_pos_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal position noise",
            horizontal_pos_std_description,
            decimals=2,
            minimum=0.,
            default=3.,
            suffix=" m",
        )
        self._rows.addWidget(self.horizontal_pos_std)

        vertical_pos_std_description = ""
        self.vertical_pos_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for vertical position noise",
            vertical_pos_std_description,
            decimals=2,
            minimum=0.,
            default=6.,
            suffix=" m",
        )
        self._rows.addWidget(self.vertical_pos_std)

        horizontal_vel_std_description = ""
        self.horizontal_vel_std = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation for horizontal speed noise",
            horizontal_vel_std_description,
            decimals=2,
            minimum=0.,
            default=0.1,
            suffix=" m/s",
        )
        self._rows.addWidget(self.horizontal_vel_std)

        vertical_vel_std_description = ""
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
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        if self.no_sensor.isChecked():
            return True

        return True

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.link.setVisible(False)
            self.update_rate.setVisible(False)
            self.horizontal_pos_std.setVisible(False)
            self.vertical_pos_std.setVisible(False)
            self.horizontal_vel_std.setVisible(False)
            self.vertical_vel_std.setVisible(False)
        else:
            self.link.setVisible(True)
            self.update_rate.setVisible(True)
            self.horizontal_pos_std.setVisible(True)
            self.vertical_pos_std.setVisible(True)
            self.horizontal_vel_std.setVisible(True)
            self.vertical_vel_std.setVisible(True)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.nwu_fixed_link_names()
        self.link.box.addItems(body_choices)
