from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QCheckBox
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

        # Vehicle Parameters
        self.vehicle = VehicleParametersWidget(self._main)
        self._rows.addWidget(self.vehicle)

        # Aerodynamic Coefficients
        self.aero_coefs = AerodynamicsCoefficientsWidget(self._main)
        self._rows.addWidget(self.aero_coefs)

        # Control Surfaces
        self.control_surfaces = ControlSurfacesWidget(self._main)
        self._rows.addWidget(self.control_surfaces)

        self._rows.addStretch()
        self._update_enability()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        if not self.has_fixed_wing.isChecked():
            return True

        if not self.vehicle.is_valid():
            return False
        if not self.aero_coefs.is_valid():
            return False
        if not self.control_surfaces.is_valid():
            return False

        return True

    @override
    def dump_settings(self) -> dict:
        raise NotImplementedError()  # TODO

    @override
    def load_settings(self, data: dict) -> None:
        raise NotImplementedError()  # TODO

    def num_control_surfaces(self) -> int:
        return self.control_surfaces.selected.count()

    def _update_enability(self) -> None:
        checked = self.has_fixed_wing.isChecked()
        self.vehicle.setEnabled(checked)
        self.aero_coefs.setEnabled(checked)
        self.control_surfaces.setEnabled(checked)

    @pyqtSlot()
    def _on_has_fixed_wing_toggled(self) -> None:
        self._update_enability()
        self._main.signals.airframe_updated.emit()
