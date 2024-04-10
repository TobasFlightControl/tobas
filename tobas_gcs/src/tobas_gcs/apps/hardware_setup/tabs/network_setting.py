from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import TableWidget, ProgressDialog
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.drone import Drone
from wpa_supplicant_parser_py.parser import WPASupplicantParser, Network

from ....common import *
from ....utils.ssh_client import SSHClientWrapper
from .base import BaseHardwareSetupWidget


class NetworkSettingWidget(BaseHardwareSetupWidget):
    NAME = "Network Setting"
    TITLE = "Setup Network"

    COL_WIDTH = 200
    WPA_SUPPLICANT_PATH = "/etc/wpa_supplicant/wpa_supplicant.conf"

    COL_SSID = 0
    COL_PSK = 1
    LABELS = ("SSID", "PSK")

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._ssh_client = SSHClientWrapper()
        self._wpa_parser = WPASupplicantParser()

        instruction = Description(
            '1. Press "Read" button to read current network settings.\n\n'
            "2. Add the settings for your network to the list.\n\n"
            '3. Press "Write" button to reflect the changes .\n\n'
        )
        self._rows.addWidget(instruction)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._read_button = QPushButton("Read")
        self._read_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._read_button)

        self._write_button = QPushButton("Write")
        self._write_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._write_button.setEnabled(False)
        cols.addWidget(self._write_button)

        self._add_button = QPushButton("Add")
        self._add_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._add_button.setEnabled(False)
        cols.addWidget(self._add_button)

        self._remove_button = QPushButton("Remove")
        self._remove_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._remove_button.setEnabled(False)
        cols.addWidget(self._remove_button)

        cols.addStretch()

        self._table = TableWidget(0, len(self.LABELS))
        self._table.setHorizontalHeaderLabels(self.LABELS)
        for c in range(self._table.columnCount()):
            self._table.setColumnWidth(c, self.COL_WIDTH)
        self._rows.addWidget(self._table)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        self._read_button.clicked.connect(self._on_read_button_clicked)
        self._write_button.clicked.connect(self._on_write_button_clicked)
        self._add_button.clicked.connect(self._on_add_button_clicked)
        self._remove_button.clicked.connect(self._on_remove_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    def _add_row(self, ssid: str, psk: str) -> None:
        row = self._table.rowCount()
        self._table.insertRow(row)
        self._table.setItem(row, self.COL_SSID, QTableWidgetItem(ssid))
        self._table.setItem(row, self.COL_PSK, QTableWidgetItem(psk))

    def _clear(self) -> None:
        rows = self._table.rowCount()
        for _ in range(rows):
            self._table.removeRow(0)

    @pyqtSlot()
    def _on_read_button_clicked(self) -> None:
        progress = ProgressDialog(parent=self._main, title=TITLE, num_steps=3)
        progress.setCancelButton(None)
        progress.show()

        # SSH接続
        progress.setLabelText("Connecting to the Raspberry Pi.")
        try:
            self._ssh_client.connect()
        except Exception as e:
            progress.close()
            q_error(self._main, str(e))
            return
        progress.progress_step()

        # リモートファイルを開いて内容を読む
        progress.setLabelText(f"Reading {self.WPA_SUPPLICANT_PATH}.")
        config_text = self._ssh_client.sftp_read(self.WPA_SUPPLICANT_PATH)
        progress.progress_step()

        # 解析の成否に関わらず編集用ボタンを有効化
        self._write_button.setEnabled(True)
        self._add_button.setEnabled(True)
        self._remove_button.setEnabled(True)

        # テキストを解析
        progress.setLabelText(f"Parsing {self.WPA_SUPPLICANT_PATH}.")
        try:
            self._wpa_parser.parse_from_text(config_text)
        except Exception as e:
            progress.close()
            q_error(self._main, f"Failed to parse network configuration:\n\n{e}")
            return

        # 現在の設定をテーブルに反映
        self._clear()
        for network in self._wpa_parser.networks:
            self._add_row(network.ssid, network.psk)
        progress.progress_step()

        progress.close()
        q_info(self._main, "Network configuration is read successfully.")

    @pyqtSlot()
    def _on_write_button_clicked(self) -> None:
        progress = ProgressDialog(parent=self._main, title=TITLE, num_steps=3)
        progress.setCancelButton(None)
        progress.show()

        # SSH接続
        progress.setLabelText("Connecting to the Raspberry Pi.")
        try:
            self._ssh_client.connect()
        except Exception as e:
            progress.close()
            q_error(self._main, str(e))
            return
        progress.progress_step()

        # WPA Parserにテーブルの内容を反映
        self._wpa_parser.networks.clear()
        for row in range(self._table.rowCount()):
            ssid = self._table.item(row, self.COL_SSID).text()
            psk = self._table.item(row, self.COL_PSK).text()
            self._wpa_parser.networks.append(Network(ssid, psk))

        # 設定を書き込む
        progress.setLabelText(f"Writing {self.WPA_SUPPLICANT_PATH}.")
        try:
            self._ssh_client.sftp_write_super(self.WPA_SUPPLICANT_PATH, self._wpa_parser.text())
        except Exception as e:
            progress.close()
            q_error(self._main, str(e))
            return
        progress.progress_step()

        # Wi-Fiを再起動
        progress.setLabelText("Restarting network.")
        command = "wpa_cli -i wlan0 reconfigure"
        # command = "systemctl restart dhcpcd.service"  # DHCPCDサーバを再起動するとアクセスポイントの接続が中断される
        # command = "ip link set wlan0 down && ip link set wlan0 up"
        success, _, error_output = self._ssh_client.exec_command_super(command)
        if not success:
            progress.close()
            q_error(self._main, f"Failed to restart DHCPCD:\n\n{error_output}")
            return
        progress.progress_step()

        progress.close()
        q_info(self._main, "Network configuration is written successfully.")

    @pyqtSlot()
    def _on_add_button_clicked(self) -> None:
        self._add_row("", "")

    @pyqtSlot()
    def _on_remove_button_clicked(self) -> None:
        row = self._table.currentRow()
        if row < 0:
            return
        self._table.removeRow(row)
