from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import os
import os.path as osp
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from overrides import override
from tobas_rqt_tools.widgets import Widget
from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.path import get_catkin_ws_path
from tobas_rqt_tools.roslaunch import create_launcher

from ...common import *
from ...utils.ssh_client import SSHClientWrapper
from ...utils.system import kill_gazebo


class SimulationWidget(Widget):
    NAME = "Simulation"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40
    WAIT_FOR_GAZEBO_SERVICE = 30.0  # [s]
    WAIT_GAZEBO_TO_OPEN = 3.0  # [s]

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

        self.setEnabled(False)  # configパッケージが読み込まれたら有効化

        self._ssh_client = SSHClientWrapper()
        self._config_pkg_path = None
        self._gazebo_launcher = None
        self._bringup_launcher = None

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._start_button)

        self._terminate_button = QPushButton("Terminate")
        self._terminate_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._terminate_button.setEnabled(False)
        cols.addWidget(self._terminate_button)

        cols.addStretch()

        # TODO: Wind Parameters

        rows.addStretch()

    @override
    def close(self) -> bool:
        if self._gazebo_launcher is not None:
            self._gazebo_launcher.shutdown()
        if self._bringup_launcher is not None:
            self._bringup_launcher.shutdown()

        return super().close()

    def define_connections(self) -> None:
        self._main.signals.config_pkg_updated.connect(self._on_config_pkg_updated)
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._terminate_button.clicked.connect(self._on_terminate_button_clicked)

    @pyqtSlot(str)
    def _on_config_pkg_updated(self, config_pkg_path: str) -> None:
        self._config_pkg_path = config_pkg_path

        self.setEnabled(True)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        config_pkg_name = osp.basename(self._config_pkg_path)

        # SSH接続
        rospy.loginfo("Connecting to the Raspberry Pi.")
        try:
            self._ssh_client.connect()
        except Exception as e:
            q_error(self._main, str(e))
            return

        # Build config package
        rospy.loginfo("Building Tobas configuration package.")
        os.chdir(self._config_pkg_path)
        if os.system(f"catkin build {config_pkg_name}") != 0:
            q_error(self._main, "Failed to build Tobas package.")
            return

        # Tobasパッケージのパスを追加
        os.environ["ROS_PACKAGE_PATH"] = self._config_pkg_path + ":" + os.environ["ROS_PACKAGE_PATH"]
        print(os.environ["ROS_PACKAGE_PATH"])

        # Stop tobas_real.service
        rospy.loginfo("Stopping tobas_real.service.")
        command = "systemctl stop tobas_real.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to stop tobas_real.service:\n\n{error_output}")
            return

        # Kill Gazebo
        rospy.loginfo("Killing Gazebo.")
        kill_gazebo()

        # Launch Gazebo
        # 一度ROSLaunchParent.shutdownを呼ぶと再開できないため，ランチャーを作り直す．
        rospy.loginfo("Launching Gazebo simulation.")
        try:
            self._gazebo_launcher = create_launcher(config_pkg_name, "gazebo.launch")
        except Exception as e:
            q_error(self._main, f"Failed to launch Gazebo simulation:\n\n{e}")
            self._reset()
            return

        # Gazeboノードの起動を待つ
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", rospy.Duration(self.WAIT_FOR_GAZEBO_SERVICE))
        except rospy.ROSException:
            q_error("Failed to connect to Gazebo server.")
            self._reset()
            return

        # Gazebo内で機体が静止するまで待つ
        # TODO: Gazebo内で一定時間経過するまで待つ
        rospy.sleep(self.WAIT_GAZEBO_TO_OPEN)

        # Launch Tobas flight controller
        # ラズパイ側でやると通信遅延が大きすぎるため，飛行制御はPC側で実行する．
        # FIXME: nodeletを有効化すると"Failed to load"エラーが出る
        rospy.loginfo("Launching Tobas flight controller.")
        self._bringup_launcher = create_launcher(config_pkg_name, "bringup.launch", args=["nodelet:=false"])

        # Start tobas_hil.service
        rospy.loginfo("Starting tobas_hil.service.")
        command = "systemctl restart tobas_hil.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to start tobas_hil.service:\n\n{error_output}")
            return

        self._start_button.setEnabled(False)
        self._terminate_button.setEnabled(True)

        q_info(self._main, "Gazebo simulation is started.")

    @pyqtSlot()
    def _on_terminate_button_clicked(self) -> None:
        if not self._reset():
            return

        q_info(self._main, "Gazebo simulation is terminated.")

    def _reset(self) -> bool:
        """初期状態に戻す．"""
        # Terminate Gazebo
        if self._gazebo_launcher is not None:
            rospy.loginfo("Terminating Gazebo simulation.")
            self._gazebo_launcher.shutdown()

        # Kill Gazebo
        rospy.loginfo("Killing Gazebo.")
        kill_gazebo()

        # Terminate Tobas flight controller
        if self._bringup_launcher is not None:
            rospy.loginfo("Terminating Tobas flight controller.")
            self._bringup_launcher.shutdown()

        # Stop tobas_hil.service
        rospy.loginfo("Stopping tobas_hil.service.")
        command = "systemctl stop tobas_hil.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to stop tobas_hil.service:\n\n{error_output}")
            return False

        # Start tobas_real.service
        rospy.loginfo("Starting tobas_real.service.")
        command = "systemctl restart tobas_real.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to restart tobas_real.service:\n\n{error_output}")
            return False

        self._start_button.setEnabled(True)
        self._terminate_button.setEnabled(False)

        return True
