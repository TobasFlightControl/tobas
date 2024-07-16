from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os.path as osp
import numpy as np
import numpy.linalg as LA
import rospy
import rospkg
from enum import Enum
from collections import deque
from overrides import override
from typing import Deque, List
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QHBoxLayout

from tobas_rqt_tools.rviz import create_rviz_frame
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import Service
from tobas_tools_py.drone import Drone
from tobas_hal_msgs.msg import MagneticField
from tobas_calibration_msgs.srv import MagCalibration, MagCalibrationRequest, MagCalibrationResponse

from ....common import PKG_NAME, WAIT_FOR_SERVER, Description
from .base import BaseHardwareSetupWidget


class MagCalibrationMethod(Enum):
    BOUNDING = 0
    SPHERE_FITTING = 1
    ELLIPSE_FITTING = 2


class MagCalibrationWidget(BaseHardwareSetupWidget):
    NAME = "Magnet Calibration"
    TITLE = "Calibrate Magnetometer"

    RVIZ_POINT_TOPIC = "rviz/magnetic_field"
    MAX_DATA_SIZE = 100000  # 8[B] * 3 * 100000 = 2400000[B] = 2.4[MB]

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        instruction = Description(
            '1. Press "Start" button.\n\n'
            "2. For each of the 6 faces of the FC, "
            "slowly rotate the FC twice around the direction of gravity with the face pointing upwards.\n\n"
            "3. Confirm that the point cloud forms a neat ellipsoid on the screen below.\n\n"
            '4. Press "Finish" button.\n\n'
        )
        self._rows.addWidget(instruction)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._start_button.clicked.connect(self._on_start_button_clicked)
        cols.addWidget(self._start_button)

        self._finish_button = QPushButton("Finish")
        self._finish_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._finish_button.setEnabled(False)
        self._finish_button.clicked.connect(self._on_finish_button_clicked)
        cols.addWidget(self._finish_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)
        cols.addWidget(self._cancel_button)

        cols.addStretch()

        pkg_path = rospkg.RosPack().get_path(PKG_NAME)
        rviz_config_path = osp.join(pkg_path, "config/mag_calibration.rviz")
        self._rviz_frame = create_rviz_frame(rviz_config_path)
        self._rows.addWidget(self._rviz_frame)

        self._rows.addStretch()

        # Point表示用ディスプレイを取得
        manager = self._rviz_frame.getManager()
        display = manager.getRootDisplayGroup().getDisplayAt(0)
        assert display.getName() == "PointStamped"

        # Rvizのトピックを指定
        display.subProp("Topic").setValue(self.RVIZ_POINT_TOPIC)

        # データバッファ関連
        self._mag_data: Deque[List[float]] = deque(maxlen=self.MAX_DATA_SIZE)
        self._point_history_length = display.subProp("History Length")

        # PubSub
        self._point_pub = rospy.Publisher(self.RVIZ_POINT_TOPIC, PointStamped, queue_size=1)
        self._mag_raw_sub: rospy.Subscriber = None

        self.setEnabled(False)

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)

    def _reset(self) -> None:
        if self._mag_raw_sub is not None:
            self._mag_raw_sub.unregister()

        self._mag_data.clear()
        self._point_history_length.setValue(0)

        self._start_button.setEnabled(True)
        self._finish_button.setEnabled(False)
        self._cancel_button.setEnabled(False)

    def _mag_raw_cb(self, msg: MagneticField) -> None:
        mag = msg.magnetic_field

        # データを追加
        self._mag_data.append([mag.x, mag.y, mag.z])

        # 表示用にPointStamped型に変換して発行
        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = "sensor"  # Rvizの設定の"Global Options/Fixed Frame"と一致させる
        point_msg.point.x = mag.x
        point_msg.point.y = mag.y
        point_msg.point.z = mag.z
        self._point_pub.publish(point_msg)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        # 地磁気トピックが正常に発行されていることを確認
        mag_topic = f"{self._drone.name}/hal/magnetic_field"
        try:
            rospy.wait_for_message(mag_topic, MagneticField, WAIT_FOR_SERVER)
        except Exception:
            q_error(self, f"Failed to get magnetometer message in {WAIT_FOR_SERVER} seconds.")
            self._reset()
            return

        # 一時的に地磁気トピックを購読開始
        self._mag_raw_sub = rospy.Subscriber(mag_topic, MagneticField, self._mag_raw_cb, queue_size=1)
        self._point_history_length.setValue(self.MAX_DATA_SIZE)

        self._start_button.setEnabled(False)
        self._finish_button.setEnabled(True)
        self._cancel_button.setEnabled(True)

        q_info(self._main, "Magnet calibration started.")

    @pyqtSlot()
    def _on_finish_button_clicked(self) -> None:
        if not self._finish_calibration():
            return

        if not self._reload_config():
            return

        self._reset()
        q_info(self._main, "Magnet calibration finished.")

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        self._reset()
        q_info(self._main, "Magnet calibration is cancelled.")

    def _finish_calibration(self) -> bool:
        calib_sc = rospy.ServiceProxy(f"{self._drone.name}/mag_calibration", MagCalibration)

        try:
            calib_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = self._create_request(MagCalibrationMethod.BOUNDING)

        try:
            res: MagCalibrationResponse = calib_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return False

        if not res.success:
            q_error(self, res.message)
            return False

        return True

    def _create_request(self, method: MagCalibrationMethod) -> MagCalibrationRequest:
        # TODO: 外れ値の除去
        # TODO: データが均一になるように間引く
        # TODO: データがきれいな楕円体を描いているかどうかをチェック

        # データを整理
        num_data = len(self._mag_data)
        data: np.ndarray = np.array(list(self._mag_data))  # NDArrayに変換
        data_split: List[np.ndarray] = [np.squeeze(v) for v in np.hsplit(data, 3)]  # 軸ごとに分割してベクトル化
        x, y, z = data_split

        # 各要素積を計算しておく
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        yz = y * z
        zx = z * x

        # リクエストを作成
        req = MagCalibrationRequest()

        # 楕円体の係数を求める
        if method == MagCalibrationMethod.BOUNDING:
            # https://okasho-engineer.com/magnetic-sensor-calibration/
            x_min: float = x.min()
            x_max: float = x.max()
            y_min: float = y.min()
            y_max: float = y.max()
            z_min: float = z.min()
            z_max: float = z.max()

            x0 = (x_min + x_max) / 2
            y0 = (y_min + y_max) / 2
            z0 = (z_min + z_max) / 2
            rx = (x_max - x_min) / 2
            ry = (y_max - y_min) / 2
            rz = (z_max - z_min) / 2
            rx2 = rx ** 2
            ry2 = ry ** 2
            rz2 = rz ** 2

            req.a_xx = 1 / rx2
            req.a_yy = 1 / ry2
            req.a_zz = 1 / rz2
            req.a_xy = 0
            req.a_yz = 0
            req.a_zx = 0
            req.b_x = -2 * x0 / rx2
            req.b_y = -2 * y0 / ry2
            req.b_z = -2 * z0 / rz2
            req.c = x0 ** 2 / rx2 + y0 ** 2 / ry2 + z0 ** 2 / rz2 - 1
        else:
            # 最小二乗法で方程式を推定: https://rikei-tawamure.com/entry/2021/10/07/211725
            # SVDは遅いが最も精度が高い: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
            req.c = -(xx + yy + zz).mean()
            ce0 = np.full((num_data,), -req.c)

            if method == MagCalibrationMethod.SPHERE_FITTING:
                # 球体でフィッティング．
                # axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
                CE = np.column_stack((xx + yy + zz, x, y, z))
                coefs: np.ndarray = LA.lstsq(CE, ce0, rcond=None)[0]

                req.a_xx = coefs[0]
                req.a_yy = coefs[0]
                req.a_zz = coefs[0]
                req.a_xy = 0
                req.a_yz = 0
                req.a_zx = 0
                req.b_x = coefs[1]
                req.b_y = coefs[2]
                req.b_z = coefs[3]
            elif method == MagCalibrationMethod.ELLIPSE_FITTING:
                # 楕円体でフィッティング．球より精密だが過学習のリスクがある．
                # axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0
                CE = np.column_stack((xx, yy, zz, 2 * xy, 2 * yz, 2 * zx, x, y, z))
                coefs: np.ndarray = LA.lstsq(CE, ce0, rcond=None)[0]

                req.a_xx = coefs[0]
                req.a_yy = coefs[1]
                req.a_zz = coefs[2]
                req.a_xy = coefs[3]
                req.a_yz = coefs[4]
                req.a_zx = coefs[5]
                req.b_x = coefs[6]
                req.b_y = coefs[7]
                req.b_z = coefs[8]
            else:
                raise RuntimeError("Invalid fitting method.")

        return req

    def _reload_config(self) -> bool:
        reload_config_sc = rospy.ServiceProxy(
            f"{self._drone.name}/magnetometer_handler/{Service.RELOAD_CONFIG}", Trigger
        )
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
