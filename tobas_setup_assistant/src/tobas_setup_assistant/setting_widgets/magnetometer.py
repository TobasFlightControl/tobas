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


class MagnetometerWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Magnetometer"
        abst_text = "地磁気センサの設定を行います．データシートを確認し，各値を入力してください．"\
            + "センサフレームは機体フレームに平行であり，値はNWU座標系で得られることを想定しています．"\
            + "Tobasのハードウェアを用いる場合は修正する必要はありません．"
        super().__init__(main, title_text, abst_text)

        link_description = "センサが取り付けられたフレームの名前．"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update rate",
            update_rate_description,
            minimum=1,
            default=100,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        gauss_noise_description = ""
        self.gauss_noise = ParamGetterWidget_SpinBox(
            "Standard deviation of additive white gaussian noise",
            gauss_noise_description,
            minimum=0,
            default=80,
            suffix=" nT",
        )
        self._rows.addWidget(self.gauss_noise)

        uniform_noise_description = ""
        self.uniform_noise = ParamGetterWidget_SpinBox(
            "Symmetric bounds of uniform noise for initial sensor bias",
            uniform_noise_description,
            minimum=0,
            default=400,
            suffix=" nT",
        )
        self._rows.addWidget(self.uniform_noise)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        return True

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.nwu_fixed_link_names()
        self.link.box.addItems(body_choices)
