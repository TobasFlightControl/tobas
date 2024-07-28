from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout

from tobas_rqt_tools.widgets import ProgressDialog
from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.roslaunch import launch
from tobas_tools_py.drone import Drone
from tobas_tools_py.package import get_tbs_config_name
from tobas_tools_py.command import kill_gazebo, build_tobas_package, source_tobas_package
from tobas_tools_py.util import is_running_under_fc_master

from ...utils.ssh_client import SSHClientWrapper
from ..base import BaseAppWidget
from .wind_parameters import WindParamsWidget


class SimulationWidget(BaseAppWidget):
    NAME = "Simulation"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40
    WAIT_GAZEBO_TO_OPEN = 3.0  # [s]

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._is_running_under_fc_master = is_running_under_fc_master()

        self._ssh_client = SSHClientWrapper()
        self._gazebo_process = None
        self._bringup_process = None

        rows = QVBoxLayout()
        self.setLayout(rows)

        button_cols = QHBoxLayout()
        rows.addLayout(button_cols)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._start_button.setEnabled(True)
        self._start_button.clicked.connect(self._on_start_button_clicked)
        button_cols.addWidget(self._start_button)

        self._terminate_button = QPushButton("Terminate")
        self._terminate_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._terminate_button.setEnabled(False)
        self._terminate_button.clicked.connect(self._on_terminate_button_clicked)
        button_cols.addWidget(self._terminate_button)

        button_cols.addStretch()

        self._wind_params = WindParamsWidget(main, drone)
        self._wind_params.setEnabled(False)
        rows.addWidget(self._wind_params)

        rows.addStretch()

        self.setEnabled(False)

    @override
    def close(self) -> bool:
        if self._gazebo_process is not None:
            self._gazebo_process.terminate()
        if self._bringup_process is not None:
            self._bringup_process.terminate()

        return super().close()

    @override
    def update_internal_data_structures(self) -> None:
        self.setEnabled(True)

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        tbs_path = self._main.tbs_path()
        config_pkg_name = get_tbs_config_name(tbs_path)

        num_steps = 11 if self._is_running_under_fc_master else 8
        progress = ProgressDialog(parent=self._main, title=self.NAME, num_steps=num_steps)
        progress.setCancelButton(None)
        progress.show()

        # SSH接続
        if self._is_running_under_fc_master:
            progress.setLabelText("Connecting to the Raspberry Pi.")
            try:
                self._ssh_client.connect()
            except Exception as e:
                progress.close()
                q_error(self._main, str(e))
                return
            progress.progress_step()

        # Build Tobas packages
        progress.setLabelText("Building Tobas packages.")
        if not build_tobas_package(tbs_path):
            progress.close()
            q_error(self._main, "Failed to build Tobas package.")
            return
        progress.progress_step()

        # Tobasパッケージのパスを追加
        progress.setLabelText("Adding Tobas package paths.")
        source_tobas_package(tbs_path)
        progress.progress_step()

        # Stop tobas_real.service
        if self._is_running_under_fc_master:
            progress.setLabelText("Stopping tobas_real.service.")
            command = "systemctl stop tobas_real.service"
            success, _, error_output = self._ssh_client.exec_command_super(command)
            if not success:
                progress.close()
                q_error(self._main, f"Failed to stop tobas_real.service:\n\n{error_output}")
                return
            progress.progress_step()

        # Kill Gazebo
        progress.setLabelText("Killing Gazebo server and client.")
        kill_gazebo()
        progress.progress_step()

        # Launch Gazebo
        # 一度ROSLaunchParent.shutdownを呼ぶと再開できないため，ランチャーを作り直す．
        progress.setLabelText("Launching Gazebo simulation.")
        self._gazebo_process = launch(config_pkg_name, "gazebo.launch")
        progress.progress_step()

        # Initialize wind parameter manager
        progress.setLabelText("Initializing wind parameter manager.")
        if not self._wind_params.initialize():
            progress.close()
            # TODO: Gazeboを落とし，tobas_real.serviceを再起動
            return
        progress.progress_step()

        # Gazebo内で機体が静止するまで待つ
        # TODO: Gazebo内で一定時間経過するまで待つ
        progress.setLabelText("Waiting for the aircraft to be static.")
        rospy.sleep(self.WAIT_GAZEBO_TO_OPEN)
        progress.progress_step()

        # Launch Tobas flight controller
        # ラズパイ側でやると通信遅延が大きすぎるため，飛行制御はPC側で実行する．
        # FIXME: nodeletを有効化すると"Failed to load"エラーが出る
        progress.setLabelText("Launching Tobas flight controller.")
        self._bringup_process = launch(config_pkg_name, "bringup.launch", {"nodelet": "false"})
        progress.progress_step()

        # Start tobas_hil.service
        if self._is_running_under_fc_master:
            progress.setLabelText("Starting tobas_hil.service.")
            command = "systemctl restart tobas_hil.service"
            success, _, error_output = self._ssh_client.exec_command_super(command)
            if not success:
                progress.close()
                q_error(self._main, f"Failed to start tobas_hil.service:\n\n{error_output}")
                return
            progress.progress_step()

        # リロード
        progress.setLabelText("Reloading.")
        self._main.update_internal_data_structures()
        progress.progress_step()

        self._start_button.setEnabled(False)
        self._terminate_button.setEnabled(True)
        self._wind_params.setEnabled(True)

        progress.close()
        q_info(self._main, "Gazebo simulation is started.")

    @pyqtSlot()
    def _on_terminate_button_clicked(self) -> None:
        num_steps = 6 if self._is_running_under_fc_master else 4
        progress = ProgressDialog(parent=self._main, title=self.NAME, num_steps=num_steps)
        progress.setCancelButton(None)
        progress.show()

        # Terminate Gazebo
        progress.setLabelText("Terminating Gazebo simulation.")
        if self._gazebo_process is not None:
            self._gazebo_process.terminate()
        progress.progress_step()

        # Kill Gazebo
        progress.setLabelText("Killing Gazebo server and client.")
        kill_gazebo()
        progress.progress_step()

        # Terminate Tobas flight controller
        progress.setLabelText("Terminating Tobas flight controller.")
        if self._bringup_process is not None:
            self._bringup_process.terminate()
        progress.progress_step()

        # Stop tobas_hil.service
        if self._is_running_under_fc_master:
            progress.setLabelText("Stopping tobas_hil.service.")
            command = "systemctl stop tobas_hil.service"
            success, _, error_output = self._ssh_client.exec_command_super(command)
            if not success:
                progress.close()
                q_error(self._main, f"Failed to stop tobas_hil.service:\n\n{error_output}")
                return
            progress.progress_step()

        # Start tobas_real.service
        if self._is_running_under_fc_master:
            progress.setLabelText("Starting tobas_real.service.")
            command = "systemctl restart tobas_real.service"
            success, _, error_output = self._ssh_client.exec_command_super(command)
            if not success:
                progress.close()
                q_error(self._main, f"Failed to restart tobas_real.service:\n\n{error_output}")
                return
            progress.progress_step()

        # リロード
        progress.setLabelText("Reloading.")
        self._main.update_internal_data_structures()
        progress.progress_step()

        self._start_button.setEnabled(True)
        self._terminate_button.setEnabled(False)
        self._wind_params.setEnabled(False)

        progress.close()
        q_info(self._main, "Gazebo simulation is terminated.")
