from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .base import BaseFixedWingSettingWidget

from typing import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QCheckBox, QVBoxLayout
from PyQt5.QtGui import QFont

from ...common import BODY_PSIZE
from ..base_setting import BaseSettingWidget
from .vehicle import VehicleParametersWidget
from .aero_coefs import AerodynamicCoefficientsWidget
from .control_surfaces import ControlSurfacesWidget


class FixedWingWidget(BaseSettingWidget):
    NAME = "Fixed Wing"
    TITLE_TEXT = "Define Fixed Wing"
    ABST_TEXT = (
        "Set up the fixed-wing configuration. " "Please choose a setup method and enter the required information."
    )

    DEFAULT_HAS_FIXED_WING = False

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self.has_fixed_wing = QCheckBox("Fixed-Wing Configuration")
        self.has_fixed_wing.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.has_fixed_wing.setChecked(self.DEFAULT_HAS_FIXED_WING)
        self.has_fixed_wing.toggled.connect(self._on_has_fixed_wing_toggled)
        self._rows.addWidget(self.has_fixed_wing)

        self._setting_rows = QVBoxLayout()
        self._rows.addLayout(self._setting_rows)

        # Vehicle
        self.vehicle = VehicleParametersWidget(self._main)
        self._setting_rows.addWidget(self.vehicle)

        # Aerodynamic Coefficients
        self.aero_coefs = AerodynamicCoefficientsWidget(self._main)
        self._setting_rows.addWidget(self.aero_coefs)

        # Control Surfaces
        self.control_surfaces = ControlSurfacesWidget(self._main)
        self._setting_rows.addWidget(self.control_surfaces)

        self._rows.addStretch()
        self._set_enabled(self.DEFAULT_HAS_FIXED_WING)

    @override
    def update_internal_data_structures(self) -> None:
        self.control_surfaces.update_internal_data_structures()

    @override
    def is_valid(self) -> bool:
        if not self.has_fixed_wing.isChecked():
            return True

        for i in range(self._setting_rows.count()):
            widget: BaseFixedWingSettingWidget = self._setting_rows.itemAt(i).widget()
            if not widget.is_valid():
                return False

        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()

        res[self.has_fixed_wing.text()] = self.has_fixed_wing.isChecked()

        for i in range(self._setting_rows.count()):
            widget: BaseFixedWingSettingWidget = self._setting_rows.itemAt(i).widget()
            res[widget.NAME] = widget.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self.has_fixed_wing.setChecked(data[self.has_fixed_wing.text()])

        for i in range(self._setting_rows.count()):
            widget: BaseFixedWingSettingWidget = self._setting_rows.itemAt(i).widget()
            widget.load_settings(data[widget.NAME])

    def num_control_surfaces(self) -> int:
        return self.control_surfaces.selected.count()

    def _set_enabled(self, enabled: bool) -> None:
        for i in range(self._setting_rows.count()):
            widget: BaseFixedWingSettingWidget = self._setting_rows.itemAt(i).widget()
            widget.setEnabled(enabled)

    @pyqtSlot(bool)
    def _on_has_fixed_wing_toggled(self, checked: bool) -> None:
        self._set_enabled(checked)
