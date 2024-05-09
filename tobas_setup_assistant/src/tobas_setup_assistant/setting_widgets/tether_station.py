from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..common import *


class TetherStationWidget(BaseSettingWidget):
    NAME = "Tether Station"

    DEFAULT_EQUIPPED = False

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Tether Station"
        abst_text = (
            "A Tether Station is a device that connects a drone to the ground with a constantly tensioned cable. "
            "It can prevent the drone from straying by using an emergency brake and "
            "extend flight duration by providing power through the cable."
        )
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox("Tether Station Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(self.DEFAULT_EQUIPPED)
        self._rows.addWidget(self._equipped)

        # Enable, Disableを一括で管理するために，設定ウィジェットを全て1つのウィジェットの子にする．
        self._config = QWidget()
        self._config.setEnabled(self.DEFAULT_EQUIPPED)
        self._rows.addWidget(self._config)

        config_rows = QVBoxLayout()
        self._config.setLayout(config_rows)

        link_description = "The name of the link to which the end of the cable is attached."
        self.link = ParamGetterWidget_ComboBox("Link Name", link_description)
        config_rows.addWidget(self.link)

        drone_end_description = "The connection point between the cable and the drone wrt. the selected link frame."
        self.drone_end = ParamGetterWidget_Vector3d("Connection Point (Drone)", drone_end_description)
        config_rows.addWidget(self.drone_end)

        world_end_description = "The connection point between the cable and the ground wrt. the world frame."
        self.world_end = ParamGetterWidget_Vector3d("Connection Point (Ground)", world_end_description)
        config_rows.addWidget(self.world_end)

        tension_description = "The constant tension of the cable."
        self.tension = ParamGetterWidget_DoubleSpinBox(
            "Tension", tension_description, decimals=2, minimum=0.0, default=1.0, suffix=" N"
        )
        config_rows.addWidget(self.tension)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._equipped.toggled.connect(self._on_equipped_toggled)
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

    @override
    def is_valid(self) -> bool:
        if not self.equipped():
            return True

        return True

    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @pyqtSlot(bool)
    def _on_equipped_toggled(self, checked: bool) -> None:
        self._config.setEnabled(checked)

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        self.link.set_choices(self._main.urdf_parser.link_names_available_in_gazebo())
