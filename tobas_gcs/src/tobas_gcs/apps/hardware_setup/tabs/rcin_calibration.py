from __future__ import annotations
from typing import TYPE_CHECKING

from PyQt5.QtGui import QPaintEvent


if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from abc import abstractmethod
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.layouts import create_fixed_width_vboxlayout
from tobas_rqt_tools.utils import remap, place_center
from tobas_tools_py.constants import *
from tobas_msgs.msg import RCInputError
from tobas_calibration_msgs.msg import RCInput
from tobas_calibration_msgs.srv import RCInputCalibration, RCInputCalibrationRequest, RCInputCalibrationResponse

from ....common import *
from .base import BaseHardwareSetupWidget


class RangeWidget(QWidget):
    LINE_WIDTH = 3
    TEXT_PSIZE = 10

    def __init__(self, minimum: int, maximum: int, text: str = None, parent: QWidget = None) -> None:
        assert minimum < maximum

        super().__init__(parent)

        self._minimum = minimum
        self._maximum = maximum
        self._text = text

        self._value = None
        self._lower = maximum
        self._upper = minimum

    def get_value(self) -> int:
        return self._value

    def set_value(self, value: int) -> None:
        self._value = value
        if value < self._lower:
            self._lower = value
        if value > self._upper:
            self._upper = value
        self.update()

    def get_lower(self) -> int:
        return self._lower

    def set_lower(self, lower: int) -> None:
        self._lower = lower

    def get_upper(self) -> int:
        return self._upper

    def set_upper(self, upper: int) -> None:
        self._upper = upper

    def get_middle(self) -> int:
        return (self._lower + self._upper) // 2

    def clear(self) -> None:
        self._value = None
        self._lower = self._maximum
        self._upper = self._minimum
        self.update()

    @abstractmethod
    def paintEvent(self, event: QPaintEvent) -> None:
        raise NotImplementedError()


class HRangeWidget(RangeWidget):
    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        if self._lower < self._upper:
            # バーの位置を計算
            lower_pos = remap(self._lower, self._minimum, self._maximum, 0, self.width())
            upper_pos = remap(self._upper, self._minimum, self._maximum, 0, self.width())

            # 最小値と最大値の間を緑色で塗る
            painter.setBrush(Qt.green)
            painter.drawRect(lower_pos, 0, upper_pos - lower_pos, self.height())

            # 最小値と最大値の位置に黒色の線を描画
            painter.setPen(QPen(Qt.black, self.LINE_WIDTH))
            painter.drawLine(lower_pos, 0, lower_pos, self.height())
            painter.drawLine(upper_pos, 0, upper_pos, self.height())

        if self._value is not None:
            # バーの位置を計算
            value_pos = remap(self._value, self._minimum, self._maximum, 0, self.width())

            # 現在値の位置に赤色の線を描画
            painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
            painter.drawLine(value_pos, 0, value_pos, self.height())

        # テキストを描画
        if self._text is not None:
            painter.setPen(Qt.gray)
            painter.setFont(QFont("Default", self.TEXT_PSIZE))
            painter.drawText(QRect(0, 0, self.width(), self.height()), Qt.AlignCenter, self._text)

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()


class VRangeWidget(RangeWidget):
    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        if self._lower < self._upper:
            # バーの位置を計算
            lower_pos = remap(self._lower, self._minimum, self._maximum, 0, self.height())
            upper_pos = remap(self._upper, self._minimum, self._maximum, 0, self.height())

            # 最小値と最大値の間を緑色で塗る
            painter.setBrush(Qt.green)
            painter.drawRect(0, lower_pos, self.width(), upper_pos - lower_pos)

            # 最小値と最大値の位置に黒色の線を描画
            painter.setPen(QPen(Qt.black, self.LINE_WIDTH))
            painter.drawLine(0, lower_pos, self.width(), lower_pos)
            painter.drawLine(0, upper_pos, self.width(), upper_pos)

        if self._value is not None:
            # バーの位置を計算
            value_pos = remap(self._value, self._minimum, self._maximum, 0, self.height())

            # 現在値の位置に赤色の線を描画
            painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
            painter.drawLine(0, value_pos, self.width(), value_pos)

        # テキストを描画
        if self._text is not None:
            # フォントを設定
            painter.setPen(Qt.gray)
            painter.setFont(QFont("Default", self.TEXT_PSIZE))

            # ペインターの回転と移動を設定
            painter.translate(self.width() / 2, self.height() / 2)
            painter.rotate(90)

            # 回転した状態でテキストを描画
            text_rect = QRect(-self.height() / 2, -self.width() / 2, self.height(), self.width())
            painter.drawText(text_rect, Qt.AlignCenter, self._text)

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()


class RcinCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Radio Calibration"
    TITLE = "Calibrate RC Input"

    PWM_MIN = 900
    PWM_MAX = 2100
    RANGE_SIDE_SHORT = 30
    RANGE_SIDE_LONG = 300

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        instruction = Description(
            '1. Press "Start" button.\n\n'
            "2. For each channel, operate the stick or switch to ensure it covers the entire range. "
            "If the stick's movement is opposite to that of the bar, adjust the transmitter settings accordingly.\n\n"
            '3. Press "Finish" button.\n\n'
        )
        self._rows.addWidget(instruction)

        cols1 = QHBoxLayout()
        self._rows.addLayout(cols1)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols1.addWidget(self._start_button)

        self._finish_button = QPushButton("Finish")
        self._finish_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._finish_button.setEnabled(False)
        cols1.addWidget(self._finish_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        cols1.addWidget(self._cancel_button)

        cols1.addStretch()

        self._rows.addSpacing(50)

        bar_cols = QHBoxLayout()
        self._rows.addLayout(bar_cols)

        # Roll, Pitch, Yaw, Thrust
        bar_rows = create_fixed_width_vboxlayout(320, bar_cols)

        self._roll_range = HRangeWidget(self.PWM_MIN, self.PWM_MAX)
        self._roll_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(QLabel(f"Roll (CH{RCIN_ROLL + 1})"), bar_rows)
        place_center(self._roll_range, bar_rows)

        pitch_thrust_cols = QHBoxLayout()
        bar_rows.addLayout(pitch_thrust_cols)

        self._pitch_range = VRangeWidget(self.PWM_MIN, self.PWM_MAX)
        self._pitch_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        pitch_thrust_cols.addWidget(self._pitch_range)

        pitch_thrust_cols.addWidget(QLabel(f"Pitch (CH{RCIN_PITCH + 1})"))
        pitch_thrust_cols.addStretch()
        pitch_thrust_cols.addWidget(QLabel(f"Thrust (CH{RCIN_THRUST + 1})"))

        self._thrust_range = VRangeWidget(self.PWM_MIN, self.PWM_MAX)
        self._thrust_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        pitch_thrust_cols.addWidget(self._thrust_range)

        self._yaw_range = HRangeWidget(self.PWM_MIN, self.PWM_MAX)
        self._yaw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._yaw_range, bar_rows)
        place_center(QLabel(f"Yaw (CH{RCIN_YAW + 1})"), bar_rows)

        bar_cols.addSpacing(30)

        bar_grid = QGridLayout()
        bar_cols.addLayout(bar_grid)

        # Mode
        bar_grid.addWidget(QLabel(f"Mode (CH{RCIN_MODE + 1})"), 0, 0)
        self._mode_range = HRangeWidget(
            self.PWM_MIN, self.PWM_MAX, "Program" + " " * 15 + "Stabilize" + " " * 15 + "Acrobat"
        )
        self._mode_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._mode_range, 0, 1)

        # E-Stop
        bar_grid.addWidget(QLabel(f"E-Stop (CH{RCIN_ESTOP + 1})"), 1, 0)
        self._estop_range = HRangeWidget(self.PWM_MIN, self.PWM_MAX, "ON" + " " * 60 + "OFF")
        self._estop_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._estop_range, 1, 1)

        # GPSw
        bar_grid.addWidget(QLabel(f"GPSw (CH{RCIN_GPSW + 1})"), 2, 0)
        self._gpsw_range = HRangeWidget(self.PWM_MIN, self.PWM_MAX, "ON" + " " * 60 + "OFF")
        self._gpsw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        bar_grid.addWidget(self._gpsw_range, 2, 1)

        bar_cols.addStretch()
        self._rows.addStretch()

        self._rcin_sub = None

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._finish_button.clicked.connect(self._on_finish_button_clicked)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)

    def _reset(self) -> None:
        self._roll_range.clear()
        self._pitch_range.clear()
        self._yaw_range.clear()
        self._thrust_range.clear()
        self._mode_range.clear()
        self._estop_range.clear()
        self._gpsw_range.clear()

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

        if self._rcin_sub is not None:
            self._rcin_sub.unregister()

    def _cancel(self) -> None:
        calib_cancel_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/rcin_calibration/cancel", Trigger)
        try:
            calib_cancel_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return

        try:
            res: TriggerResponse = calib_cancel_sc.call(TriggerRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if not res.success:
            q_error(self, res.message)
            return

        self._reset()

    def _rcin_cb(self, msg: RCInput) -> None:
        if msg.error.error != RCInputError.E_NO_ERROR:
            q_error(self, "RC signal is not received.")
            self._cancel()
            return

        self._roll_range.set_value(msg.data[RCIN_ROLL])
        self._pitch_range.set_value(msg.data[RCIN_PITCH])
        self._yaw_range.set_value(msg.data[RCIN_YAW])
        self._thrust_range.set_value(msg.data[RCIN_THRUST])
        self._mode_range.set_value(msg.data[RCIN_MODE])
        self._estop_range.set_value(msg.data[RCIN_ESTOP])
        self._gpsw_range.set_value(msg.data[RCIN_GPSW])

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        calib_start_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/rcin_calibration/start", Trigger)
        try:
            calib_start_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return

        try:
            res: TriggerResponse = calib_start_sc.call(TriggerRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if not res.success:
            q_error(self, res.message)
            return

        # RC入力が正常に発行されていることを確認
        rcin_topic = f"{self._drone.drone_name}/rcin_calibration/rc_input_raw"
        try:
            rcin_msg: RCInput = rospy.wait_for_message(rcin_topic, RCInput, self.WAIT_FOR_SERVER)
        except Exception:
            q_error(self, f"Failed to get RC input message in {self.WAIT_FOR_SERVER} seconds.")
            self._cancel()
            return
        if rcin_msg.error.error != RCInputError.E_NO_ERROR:
            q_error(self, "RC signal is not received.")
            self._cancel()
            return

        # 購読するトピックを更新
        self._rcin_sub = rospy.Subscriber(rcin_topic, RCInput, self._rcin_cb, queue_size=1)

        self._start_button.setEnabled(False)
        self._finish_button.setEnabled(True)
        self._cancel_button.setEnabled(True)

        q_info(self._main, "Radio calibration started.")

    @pyqtSlot()
    def _on_finish_button_clicked(self) -> None:
        if not self._finish_calibration():
            return

        if not self._reload_config():
            return

        self._reset()
        q_info(self._main, "Radio calibration finished.")

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        self._cancel()
        q_info(self._main, "Radio calibration is cancelled.")

    def _finish_calibration(self) -> bool:
        calib_finish_sc = rospy.ServiceProxy(f"{self._drone.drone_name}/rcin_calibration/finish", RCInputCalibration)
        try:
            calib_finish_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = RCInputCalibrationRequest()
        req.roll_left = self._roll_range.get_lower()
        req.roll_right = self._roll_range.get_upper()
        req.pitch_down = self._pitch_range.get_upper()
        req.pitch_up = self._pitch_range.get_lower()
        req.yaw_left = self._yaw_range.get_lower()
        req.yaw_right = self._yaw_range.get_upper()
        req.thrust_down = self._thrust_range.get_upper()
        req.thrust_up = self._thrust_range.get_lower()
        req.mode_program = self._mode_range.get_lower()
        req.mode_stabilize = self._mode_range.get_middle()
        req.mode_acrobat = self._mode_range.get_upper()
        req.estop_on = self._estop_range.get_lower()
        req.estop_off = self._estop_range.get_upper()
        req.gpsw_on = self._gpsw_range.get_lower()
        req.gpsw_off = self._gpsw_range.get_upper()

        try:
            res: RCInputCalibrationResponse = calib_finish_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True

    def _reload_config(self) -> bool:
        reload_config_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/rcin_handler/reload_config", Trigger)
        try:
            reload_config_sc.wait_for_service(self.WAIT_FOR_SERVER)
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
