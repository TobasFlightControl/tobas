from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import DoubleSpinBox
from tobas_rqt_tools.messages import q_info, q_error
from tobas_calibration_msgs.srv import AdcCalibration, AdcCalibrationRequest, AdcCalibrationResponse

from ....common import *
from .base import BaseHardwareSetupWidget


class AdcCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "ADC Calibration"
    TITLE = "Calibrate Analog-Digital Converter"

    WIDTH = 100

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        instruction = Description(
            "1. Ensure the battery and the ADC port is correctly connected.\n\n"
            "2. Input the current battery voltage.\n\n"
            '3. Press "Start" button.\n\n'
        )
        self._rows.addWidget(instruction)

        cols1 = QHBoxLayout()
        self._rows.addLayout(cols1)

        cols1.addWidget(QLabel("Voltage:"))

        self._voltage = DoubleSpinBox()
        self._voltage.setSuffix(" V")
        self._voltage.setFixedWidth(self.WIDTH)
        self._voltage.setMinimum(0.0)
        cols1.addWidget(self._voltage)

        cols1.addStretch()

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._rows.addWidget(self._start_button)

        self._rows.addSpacing(50)

        cols2 = QHBoxLayout()
        self._rows.addLayout(cols2)

        cols2.addWidget(QLabel("ADC Coefficient:"))

        self._adc_coef = QLineEdit()
        self._adc_coef.setFixedWidth(self.WIDTH)
        self._adc_coef.setReadOnly(True)
        self._adc_coef.setFocusPolicy(Qt.NoFocus)
        cols2.addWidget(self._adc_coef)

        cols2.addStretch()

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._start_button.clicked.connect(self._on_start_button_clicked)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        adc_calib_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/adc_calibration", AdcCalibration)

        try:
            adc_calib_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, "Failed to connect to the calibration server.")
            return

        req = AdcCalibrationRequest()
        req.voltage = self._voltage.value()

        res: AdcCalibrationResponse = adc_calib_sc.call(req)
        if not res.success:
            q_error(self, res.message)
            return

        self._adc_coef.setText(f"{res.coefficient:.2f}")

        q_info(self._main, "ADC calibration finished.")
