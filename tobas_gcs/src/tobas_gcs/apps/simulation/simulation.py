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

from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.path import get_catkin_ws_path
from tobas_rqt_tools.roslaunch import create_launcher

from ...common import *
from ...utils.ssh_client import SSHClientWrapper
from ...utils.system import kill_gazebo


class SimulationWidget(QWidget):
    NAME = "Simulation"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40
    WAIT_GAZEBO_TO_OPEN = 3.0  # [s]

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
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
        catkin_ws_path = get_catkin_ws_path(self._config_pkg_path)
        config_pkg_name = osp.basename(self._config_pkg_path)
        setup_bash_path = osp.join(catkin_ws_path, "devel/setup.bash")

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

        # Source catkin workspace
        rospy.loginfo("Sourcing catkin workspace.")
        if os.system(f"bash -c 'source {setup_bash_path}'") != 0:
            q_error(self._main, f"Failed to source catkin workspace.")
            return

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
        self._gazebo_launcher = create_launcher(config_pkg_name, "gazebo.launch")
        rospy.wait_for_service("/gazebo/get_world_properties")  # Gazeboの起動を待つ
        rospy.sleep(self.WAIT_GAZEBO_TO_OPEN)  # TODO: Gazebo内で一定時間経過するまで待つ

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
        # Terminate Gazebo
        rospy.loginfo("Terminating Gazebo simulation.")
        self._gazebo_launcher.shutdown()

        # Kill Gazebo
        rospy.loginfo("Killing Gazebo.")
        kill_gazebo()

        # Terminate Tobas flight controller
        rospy.loginfo("Terminating Tobas flight controller.")
        self._bringup_launcher.shutdown()

        # Stop tobas_hil.service
        rospy.loginfo("Stopping tobas_hil.service.")
        command = "systemctl stop tobas_hil.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to stop tobas_hil.service:\n\n{error_output}")
            return

        # Start tobas_real.service
        rospy.loginfo("Starting tobas_real.service.")
        command = "systemctl restart tobas_real.service"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            q_error(self._main, f"Failed to restart tobas_real.service:\n\n{error_output}")
            return

        self._start_button.setEnabled(True)
        self._terminate_button.setEnabled(False)

        q_info(self._main, "Gazebo simulation is terminated.")
