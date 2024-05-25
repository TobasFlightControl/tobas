from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from overrides import override
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QHBoxLayout

from tobas_rqt_tools.widgets import ProgressDialog
from tobas_rqt_tools.widgets import DoubleSpinBox
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone
from tobas_calibration_msgs.srv import AdcCalibration, AdcCalibrationRequest, AdcCalibrationResponse

from ....common import WAIT_FOR_SERVER, Description
from .base import BaseHardwareSetupWidget


class AdcCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "ADC Calibration"
    TITLE = "Calibrate Analog-Digital Converter"

    WIDTH = 100

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

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
        self._start_button.clicked.connect(self._on_start_button_clicked)
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

        self.setEnabled(False)

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        progress = ProgressDialog(parent=self._main, title=self.NAME, num_steps=2)
        progress.setCancelButton(None)
        progress.show()

        progress.setLabelText("Calibrating.")
        if not self._calibrate():
            progress.close()
            return
        progress.progress_step()

        progress.setLabelText("Reloading.")
        if not self._reload_config():
            progress.close()
            return
        progress.progress_step()

        progress.close()
        q_info(self._main, "ADC calibration finished.")

    def _calibrate(self) -> bool:
        adc_calib_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/adc_calibration", AdcCalibration)
        try:
            adc_calib_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = AdcCalibrationRequest()
        req.voltage = self._voltage.value()

        try:
            res: AdcCalibrationResponse = adc_calib_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        self._adc_coef.setText(f"{res.coefficient:.2f}")

        return True

    def _reload_config(self) -> bool:
        reload_config_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/battery_handler/reload_config", Trigger)
        try:
            reload_config_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        try:
            res: TriggerResponse = reload_config_sc.call(TriggerRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True
