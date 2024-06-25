from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant
    from ..parameter_getters import ParamGetterWidget

from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from ..common import SENSOR_OFFSET_DESCRIPTION
from ..parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_Vector3d
from .base_setting import BaseSettingWidget


class MagnetometerWidget(BaseSettingWidget):
    NAME = "Magnetometer"
    TITLE_TEXT = "Define Magnetometer"
    ABST_TEXT = ""  # TODO

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        self.offset = ParamGetterWidget_Vector3d("Offset", SENSOR_OFFSET_DESCRIPTION, suffix=" m")
        self._param_rows.addWidget(self.offset)

        update_rate_description = ""
        self.update_rate = ParamGetterWidget_SpinBox(
            "Update Rate", update_rate_description, minimum=1, default=80, suffix=" Hz"
        )
        self._param_rows.addWidget(self.update_rate)

        gauss_noise_description = ""
        self.gauss_noise = ParamGetterWidget_SpinBox(
            "Standard Deviation of Additive White Gaussian Noise",
            gauss_noise_description,
            minimum=0,
            default=80,
            suffix=" nT",
        )
        self._param_rows.addWidget(self.gauss_noise)

        uniform_noise_description = ""
        self.uniform_noise = ParamGetterWidget_SpinBox(
            "Symmetric Bounds of Uniform Noise for Initial Bias",
            uniform_noise_description,
            minimum=0,
            default=400,
            suffix=" nT",
        )
        self._param_rows.addWidget(self.uniform_noise)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()
        return res

    @override
    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])

    def equipped(self) -> bool:
        return True
