from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget

from ...parameter_getters import *
from ...common import *
from ..base_setting import BaseSettingWidget
from .vehicle import VehicleParametersWidget
from .aero_coefs import AerodynamicsCoefficientsWidget
from .control_surfaces import ControlSurfacesWidget


class FixedWingWidget(BaseSettingWidget):

    NAME = "Fixed Wing"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Fixed Wing"
        abst_text = "固定翼の設定を行います．"\
            + "設定方法を選択し，必要事項を入力してください．"
        super().__init__(main, title_text, abst_text)

        self.has_fixed_wing = QCheckBox("Fixed-Wing Configuration")
        self.has_fixed_wing.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.has_fixed_wing.setChecked(False)
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

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.has_fixed_wing.toggled.connect(self._on_has_fixed_wing_toggled)
        self.vehicle.define_connections()
        self.aero_coefs.define_connections()
        self.control_surfaces.define_connections()

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

    def _update_visibility(self) -> None:
        if self.has_fixed_wing.isChecked():
            self.vehicle.setVisible(True)
            self.aero_coefs.setVisible(True)
            self.control_surfaces.setVisible(True)
        else:
            self.vehicle.setVisible(False)
            self.aero_coefs.setVisible(False)
            self.control_surfaces.setVisible(False)

    @pyqtSlot()
    def _on_has_fixed_wing_toggled(self) -> None:
        self._update_visibility()
        self._main.signals.airframe_updated.emit()
