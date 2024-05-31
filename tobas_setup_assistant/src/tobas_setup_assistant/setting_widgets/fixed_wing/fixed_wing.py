from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .base import BaseFixedWingSettingWidget

from overrides import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QCheckBox, QVBoxLayout
from PyQt5.QtGui import QFont

from ...common import BODY_PSIZE
from ..base_setting import BaseSettingWidget
from .vehicle import VehicleParametersWidget
from .aero_coefs import AerodynamicsCoefficientsWidget
from .control_surfaces import ControlSurfacesWidget


class FixedWingWidget(BaseSettingWidget):
    NAME = "Fixed Wing"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Fixed Wing"
        abst_text = (
            "Set up the fixed-wing configuration. " "Please choose a setup method and enter the required information."
        )
        super().__init__(main, title_text, abst_text)

        self.has_fixed_wing = QCheckBox("Fixed-Wing Configuration")
        self.has_fixed_wing.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.has_fixed_wing.setChecked(False)
        self.has_fixed_wing.toggled.connect(self._on_has_fixed_wing_toggled)
        self._rows.addWidget(self.has_fixed_wing)

        self._setting_rows = QVBoxLayout()
        self._rows.addLayout(self._setting_rows)

        # Vehicle
        self.vehicle = VehicleParametersWidget(self._main)
        self._setting_rows.addWidget(self.vehicle)

        # Aerodynamic Coefficients
        self.aero_coefs = AerodynamicsCoefficientsWidget(self._main)
        self._setting_rows.addWidget(self.aero_coefs)

        # Control Surfaces
        self.control_surfaces = ControlSurfacesWidget(self._main)
        self._setting_rows.addWidget(self.control_surfaces)

        self._rows.addStretch()
        self._update_enability()

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
            widget.load_settings(data)

    def num_control_surfaces(self) -> int:
        return self.control_surfaces.selected.count()

    def _update_enability(self) -> None:
        checked = self.has_fixed_wing.isChecked()
        for i in range(self._setting_rows.count()):
            widget: BaseFixedWingSettingWidget = self._setting_rows.itemAt(i).widget()
            widget.setEnabled(checked)

    @pyqtSlot()
    def _on_has_fixed_wing_toggled(self) -> None:
        self._update_enability()
        self._main.signals.airframe_updated.emit()
