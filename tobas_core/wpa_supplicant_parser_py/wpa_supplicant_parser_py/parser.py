from typing import List

from .network import Network, KeyManagement
from .country_code import CountryCode


class WPASupplicantParser:

    DEFAULT_COUNTRY = CountryCode.Japan
    DEFAULT_CTRL_INTERFACE = "DIR=/var/run/wpa_supplicant GROUP=netdev"
    DEFAULT_UPDATE_CONFIG = True

    COUNTRY_PREFIX = "country="
    CTRL_INTERFACE_PREFIX = "ctrl_interface="
    UPDATE_CONFIG_PREFIX = "update_config="

    START_NETWORK_BLOCK = "network={"
    STOP_NETWORK_BLOCK = "}"
    SSID_PREFIX = "ssid="
    PSK_PREFIX = "psk="
    KEY_MGMT_PREFIX = "key_mgmt="
    PRIORITY_PREFIX = "priority="

    def __init__(self) -> None:
        self.country: CountryCode = self.DEFAULT_COUNTRY
        self.ctrl_interface: str = self.DEFAULT_CTRL_INTERFACE
        self.update_config: bool = self.DEFAULT_UPDATE_CONFIG
        self.networks: List[Network] = []

    def clear(self) -> None:
        self.country = self.DEFAULT_COUNTRY
        self.ctrl_interface = self.DEFAULT_CTRL_INTERFACE
        self.update_config = self.DEFAULT_UPDATE_CONFIG
        self.networks = []

    def parse_from_text(self, text: str) -> None:
        self.clear()

        lines = text.splitlines()
        network = Network()
        in_network_block = False

        for line in lines:
            # 行頭または行末の空白を削除
            line = line.strip()

            # 空行またはコメント行をスキップ
            if not line or line.startswith("#"):
                continue

            # country
            if line.startswith(self.COUNTRY_PREFIX):
                cc = line.split("=", 1)[1]
                for item in CountryCode:
                    if item.value == cc:
                        self.country = item
                        break
                else:
                    raise RuntimeError(f"Invalid country code: {cc}")
                continue

            # ctrl_interface
            if line.startswith(self.CTRL_INTERFACE_PREFIX):
                self.ctrl_interface = line.split("=", 1)[1]
                continue

            # update_config
            if line.startswith(self.UPDATE_CONFIG_PREFIX):
                self.update_config = bool(line.split("=", 1)[1])
                continue

            # ネットワークブロックの開始
            if line == self.START_NETWORK_BLOCK:
                network = Network()
                in_network_block = True
                continue

            # ネットワークブロックの終了
            if line == self.STOP_NETWORK_BLOCK:
                if not in_network_block:
                    raise RuntimeError("Unexpected closing bracket.")

                self.networks.append(network)
                in_network_block = False
                continue

            # ssid
            if line.startswith(self.SSID_PREFIX):
                if not in_network_block:
                    raise RuntimeError("A setting for SSID is found outside the network block.")

                network.ssid = line.split("=", 1)[1].strip('"')
                continue

            # psk
            if line.startswith(self.PSK_PREFIX):
                if not in_network_block:
                    raise RuntimeError("A setting for PSK is found outside the network block.")

                network.psk = line.split("=", 1)[1].strip('"')
                continue

            # key_mgmt
            if line.startswith(self.KEY_MGMT_PREFIX):
                if not in_network_block:
                    raise RuntimeError("A setting for key management is found outside the network block.")

                key_mgmt = line.split("=", 1)[1]
                for item in KeyManagement:
                    if item.value == key_mgmt:
                        network.key_mgmt = item
                        break
                    else:
                        raise RuntimeError("Invalid key management setting.")
                continue

            # priority
            if line.startswith(self.PRIORITY_PREFIX):
                if not in_network_block:
                    raise RuntimeError("A setting for network priority is found outside the network block.")

                network.priority = int(line.split("=", 1)[1])
                continue

    def parse_from_file(self, path: str) -> None:
        with open(path, "r") as f:
            text = f.read()
        self.parse_from_text(text)

    def text(self) -> str:
        """設定をwpa_supplicant.confのテキスト形式で返す．"""
        text = ""
        text += f"{self.COUNTRY_PREFIX}{self.country.value}\n"
        text += f"{self.CTRL_INTERFACE_PREFIX}{self.ctrl_interface}\n"
        text += f"{self.UPDATE_CONFIG_PREFIX}{int(self.update_config)}\n"

        for network in self.networks:
            text += "\n"
            text += f"{self.START_NETWORK_BLOCK}\n"
            text += f'\t{self.SSID_PREFIX}"{network.ssid}"\n'
            text += f'\t{self.PSK_PREFIX}"{network.psk}"\n'
            text += f"\t{self.KEY_MGMT_PREFIX}{network.key_mgmt.value}\n"
            text += f"\t{self.PRIORITY_PREFIX}{network.priority}\n"
            text += f"{self.STOP_NETWORK_BLOCK}\n"

        return text

    def write(self, path: str) -> None:
        with open(path, "w") as f:
            f.write(self.text())
