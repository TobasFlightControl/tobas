from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os.path as osp
import paramiko
import socket
from scp import SCPClient
from overrides import override
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_info, q_error
from wpa_supplicant_parser_py.parser import WPASupplicantParser, Network

from ....common import *
from .base import BaseHardwareSetupWidget


class NetworkSettingWidget(BaseHardwareSetupWidget):
    NAME = "Network Setting"
    TITLE = "Setup Network"

    COL_WIDTH = 200
    WPA_SUPPLICANT_PATH = "/etc/wpa_supplicant/wpa_supplicant.conf"
    WPA_SUPPLICANT_PATH_TMP = "/tmp/wpa_supplicant.conf"

    COL_SSID = 0
    COL_PSK = 1
    LABELS = ("SSID", "PSK")

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(main)

        # SSHクライアント
        # TODO: AutoAddPolicyは脆弱なので，予めサーバーのホストキーをクライアントに登録する
        self._ssh_client = paramiko.SSHClient()
        self._ssh_client.load_system_host_keys()
        self._ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

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

        self._table = QTableWidget(0, len(self.LABELS))
        self._table.setHorizontalHeaderLabels(self.LABELS)
        for c in range(self._table.columnCount()):
            self._table.setColumnWidth(c, self.COL_WIDTH)
        self._rows.addWidget(self._table)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._read_button.clicked.connect(self._on_read_button_clicked)
        self._write_button.clicked.connect(self._on_write_button_clicked)
        self._add_button.clicked.connect(self._on_add_button_clicked)
        self._remove_button.clicked.connect(self._on_remove_button_clicked)

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
        # SSH接続
        # TODO: SSH鍵認証，環境変数，秘密管理ツール等を使用して認証情報を安全に管理する
        try:
            self._ssh_client.connect(HOST_NAME, PORT, USER, LOGIN_PASSWORD)
        except paramiko.AuthenticationException:
            q_error(
                self._main,
                "Authentication failed. Please check your username or password.",
            )
            return
        except paramiko.SSHException:
            q_error(
                self._main,
                "Failed to establish an SSH connection.",
            )
            return
        except socket.error:
            q_error(
                self._main,
                "Could not connect to the server. Please check your network connection.",
            )
            return
        except Exception as e:
            q_error(self._main, f"Unexpected error occurred:\n\n{e}")
            return

        # リモートファイルを開いて内容を読む
        sftp = self._ssh_client.open_sftp()
        with sftp.file(self.WPA_SUPPLICANT_PATH, "r") as f:
            config_text = f.read().decode("utf-8")
        sftp.close()

        # 解析の成否に関わらず編集用ボタンを有効化
        self._write_button.setEnabled(True)
        self._add_button.setEnabled(True)
        self._remove_button.setEnabled(True)

        # テキストを解析
        try:
            self._wpa_parser.parse_from_text(config_text)
        except Exception as e:
            q_error(self._main, f"Failed to parse network configuration:\n\n{e}")
            return

        # 現在の設定をテーブルに反映
        self._clear()
        for network in self._wpa_parser.networks:
            self._add_row(network.ssid, network.psk)

        q_info(self._main, "Network configuration is read successfully.")

    @pyqtSlot()
    def _on_write_button_clicked(self) -> None:
        # WPA Parserにテーブルの内容を反映
        self._wpa_parser.networks.clear()
        for row in range(self._table.rowCount()):
            ssid = self._table.item(row, self.COL_SSID).text()
            psk = self._table.item(row, self.COL_PSK).text()
            self._wpa_parser.networks.append(Network(ssid, psk))

        # SFTPセッションを開始し，一時ファイルに書き込む
        sftp = self._ssh_client.open_sftp()
        try:
            with sftp.file(self.WPA_SUPPLICANT_PATH_TMP, "w") as f:
                f.write(self._wpa_parser.text())
        except Exception as e:
            q_error(self._main, f"Failed to write network configuration to the temporary file:\n\n{e}")
            return
        sftp.close()

        # SSH経由でsudoを使用して一時ファイルを目的の場所に移動させる
        command = SUDO_PREFIX + f"mv {self.WPA_SUPPLICANT_PATH_TMP} {self.WPA_SUPPLICANT_PATH}"
        _, stdout, stderr = self._ssh_client.exec_command(command)
        stdout.channel.recv_exit_status()  # コマンドの実行結果を待つ
        exit_status = stdout.channel.exit_status
        error_output = stderr.read().decode("utf-8")  # 標準エラー出力
        if exit_status != 0:
            q_error(
                self._main,
                f"Failed to write network configuration:\n\n{error_output}",
            )
            return

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
