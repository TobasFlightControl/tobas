from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from ..base_setting import BaseSettingWidget
from .robot_model_loader import RobotModelLoaderWidget


class StartWidget(BaseSettingWidget):
    NAME = "Start"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Tobas Setup Assistant"
        abst_text = (
            "The Tobas Setup Assistant is a GUI tool designed for creating configuration files "
            "needed to operate drones with Tobas. "
            "It utilizes the URDF created in the previous steps and allows for the configuration of elements "
            "not expressed in the URDF, "
            "such as propeller aerodynamics and controller settings."
        )
        super().__init__(main, title_text, abst_text)

        self.setEnabled(True)  # Startだけは初めからアクティブにしておく

        self._robot_model_loader = RobotModelLoaderWidget(main)
        self._rows.addWidget(self._robot_model_loader)

        self._rows.addStretch()

    @override
    def is_valid(self) -> bool:
        return True
