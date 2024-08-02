from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import override

from tobas_rqt_tools.widgets import ComboBox, StackedWidget
from tobas_rqt_tools.messages import q_error_named

from ..base_setting import BaseSettingWidget
from .base import BaseController
from .multirotor_pid import MultirotorPid
from .multirotor_mpc import MultirotorMpc
from .arducopter import ArduCopter
from .non_planar_pid import NonPlanarPid
from .fixed_wing_lqr import FixedWingLQR
from .custom import CustomController


class ControllerWidget(BaseSettingWidget):
    NAME = "Controller"
    TITLE_TEXT = "Setup Controller"
    ABST_TEXT = (
        "Configure the flight controller by selecting one method and setting its parameters. "
        "You can fine-tune the parameters later, "
        "so it's acceptable to leave them at their default settings initially."
    )

    CONTROLLER_TYPE_KEY = "controller_type"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._type = ComboBox()
        self._controllers = StackedWidget()

        self._type.currentIndexChanged.connect(self._controllers.setCurrentIndex)

        for controller_class in [
            MultirotorPid,
            MultirotorMpc,
            ArduCopter,
            NonPlanarPid,
            FixedWingLQR,
            CustomController,
        ]:
            self._type.addItem(controller_class.NAME)
            self._controllers.addWidget(controller_class(main))

        self._rows.addWidget(self._type)
        self._rows.addWidget(self._controllers)
        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        for i in range(self._controllers.count()):
            controller: BaseController = self._controllers.widget(i)
            controller.update_internal_data_structures()

    @override
    def is_valid(self) -> bool:
        if not self._selected().is_applicable():
            q_error_named(
                self._main,
                self.NAME,
                "The selected controller is not applicable to the airframe.",
            )
            return False

        if not self._selected().is_valid():
            return False

        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()

        res[self.CONTROLLER_TYPE_KEY] = self._type.currentText()

        for i in range(self._controllers.count()):
            controller: BaseController = self._controllers.widget(i)
            res[controller.NAME] = controller.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._type.setCurrentText(data[self.CONTROLLER_TYPE_KEY])

        for i in range(self._controllers.count()):
            controller: BaseController = self._controllers.widget(i)
            controller.load_settings(data[controller.NAME])

    @override
    def on_opened(self) -> None:
        for i in range(self._controllers.count()):
            controller: BaseController = self._controllers.widget(i)
            controller.on_opened()

            # 現在の機体設定で適用できない場合は選択肢にその旨を表示する
            if controller.is_applicable():
                self._type.setItemText(i, controller.NAME)
            else:
                self._type.setItemText(i, f"{controller.NAME} (Not Applicable)")

    def controller_pkg(self) -> str:
        return self._selected().CONTROLLER_PKG

    def takeoff_pkg(self) -> str:
        return self._selected().TAKEOFF_PKG

    def landing_pkg(self) -> str:
        return self._selected().LANDING_PKG

    def move_pkg(self) -> str:
        return self._selected().MOVE_PKG

    def stabilize_mode(self) -> str:
        return self._selected().STABLIZE_MODE

    def acrobat_mode(self) -> str:
        return self._selected().ACROBAT_MODE

    def static_parameters(self) -> dict:
        return self._selected().static_parameters()

    def _selected(self) -> BaseController:
        return self._controllers.currentWidget()
