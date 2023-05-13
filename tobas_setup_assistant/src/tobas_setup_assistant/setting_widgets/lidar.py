from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *


class LidarWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define LiDAR'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with LiDAR.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link Nane", link_description, [])
        self._rows.addWidget(self.link)

        raw_topic_description = "TODO: instruction"
        self.raw_topic = ParamGetterWidget_LineEdit(
            "Point Cloud Topic",
            raw_topic_description,
            "/head_mount_kinect/depth_registered/points")
        self._rows.addWidget(self.raw_topic)

        max_range_description = "TODO: instruction"
        self.max_range = ParamGetterWidget_DoubleSpinBox(
            "Max Range",
            max_range_description,
            minimum=0.,
            default=5.,
        )
        self._rows.addWidget(self.max_range)

        subsample_description = "TODO: instruction"
        self.subsample = ParamGetterWidget_SpinBox(
            "Point Subsample",
            subsample_description,
            minimum=0,
            default=1,
        )
        self._rows.addWidget(self.subsample)

        padding_offset_description = "TODO: instruction"
        self.padding_offset = ParamGetterWidget_DoubleSpinBox(
            "Padding Offset",
            padding_offset_description,
            minimum=0.,
            default=0.1,
        )
        self._rows.addWidget(self.padding_offset)

        padding_scale_description = "TODO: instruction"
        self.padding_scale = ParamGetterWidget_DoubleSpinBox(
            "Padding Scale",
            padding_scale_description,
            minimum=0.,
            default=0.1,
        )
        self._rows.addWidget(self.padding_scale)

        filtered_topic_description = "TODO: instruction"
        self.filtered_topic = ParamGetterWidget_LineEdit(
            "Filtered Cloud Topic",
            filtered_topic_description,
            "/head_mount_kinect/depth_registered/points")
        self._rows.addWidget(self.filtered_topic)

        max_update_rate_description = "TODO: instruction"
        self.max_update_rate = ParamGetterWidget_DoubleSpinBox(
            "Max Update Rate",
            max_update_rate_description,
            minimum=1.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.max_update_rate)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    def is_valid(self) -> bool:
        return True

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.link.setVisible(False)
            self.raw_topic.setVisible(False)
            self.max_range.setVisible(False)
            self.subsample.setVisible(False)
            self.padding_offset.setVisible(False)
            self.padding_scale.setVisible(False)
            self.filtered_topic.setVisible(False)
            self.max_update_rate.setVisible(False)
        else:
            self.link.setVisible(True)
            self.raw_topic.setVisible(True)
            self.max_range.setVisible(True)
            self.subsample.setVisible(True)
            self.padding_offset.setVisible(True)
            self.padding_scale.setVisible(True)
            self.filtered_topic.setVisible(True)
            self.max_update_rate.setVisible(True)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.nwu_fixed_link_names()
        self.link.box.addItems(body_choices)
