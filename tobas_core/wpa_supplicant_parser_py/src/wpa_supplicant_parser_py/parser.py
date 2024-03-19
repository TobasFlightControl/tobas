from typing import List
from dataclasses import dataclass
from enum import Enum


class KeyManagement(Enum):
    WPA_PSK = "WPA-PSK"  # 個人用WPA/WPA2
    WPA_EAP = "WPA-EAP"  # 企業用WPA/WPA2


class CountryCode(Enum):
    AFGHANISTAN = "AF"
    ALBANIA = "AL"
    ALGERIA = "DZ"
    AMERICAN_SAMOA = "AS"
    ANDORRA = "AD"
    ANGOLA = "AO"
    # TODO: 全ての国コードを書く (https://countrycode.org/)
    JAPAN = "JP"


@dataclass
class Network:
    ssid: str = ""
    psk: str = ""
    key_mgmt: KeyManagement = KeyManagement.WPA_PSK
    priority: int = 0

    def __repr__(self) -> str:
        res = ""
        res += f"SSID: {self.ssid}\n"
        res += f"PSK: {self.psk}\n"
        res += f"Key Management: {self.key_mgmt.value}\n"
        res += f"Priority: {self.priority}\n"
        return res


class WPASupplicantParser:

    DEFAULT_COUNTRY = CountryCode.JAPAN
    DEFAULT_CTRL_INTERFACE = "DIR=/var/run/wpa_supplicant GROUP=netdev"
    DEFAULT_UPDATE_CONFIG = True

    def __init__(self) -> None:
        self.country: CountryCode = self.DEFAULT_COUNTRY
        self.ctrl_interface: str = self.DEFAULT_CTRL_INTERFACE
        self.update_config: bool = self.DEFAULT_UPDATE_CONFIG
        self.networks: List[Network] = []

    def __repr__(self) -> str:
        res = ""
        res += f"Country: {self.country.value}\n"
        res += f"Countrol Interface: {self.ctrl_interface}\n"
        res += f"Update Configuration: {self.update_config}\n"
        res += f"Networks:\n"
        for network in self.networks:
            res += "---\n"
            res += str(network)
        res += "---\n"
        return res

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
            if line.startswith("country="):
                cc = line.split("=", 1)[1]
                for item in CountryCode:
                    if item.value == cc:
                        self.country = item
                        break
                else:
                    raise RuntimeError(f"Invalid country code: {cc}")
                continue

            # ctrl_interface
            if line.startswith("ctrl_interface="):
                self.ctrl_interface = line.split("=", 1)[1]
                continue

            # update_config
            if line.startswith("update_config="):
                self.update_config = bool(line.split("=", 1)[1])
                continue

            # ネットワークブロックの開始
            if line == "network={":
                network = Network()
                in_network_block = True
                continue

            # ネットワークブロックの終了
            if line == "}":
                if not in_network_block:
                    raise RuntimeError(f"Unexpected closing bracket.")

                self.networks.append(network)
                in_network_block = False
                continue

            # SSID
            if line.startswith("ssid="):
                if not in_network_block:
                    raise RuntimeError(f"A setting for SSID is found outside the network block.")

                network.ssid = line.split("=", 1)[1].strip('"')
                continue

            # PSK
            if line.startswith("psk="):
                if not in_network_block:
                    raise RuntimeError(f"A setting for PSK is found outside the network block.")

                network.psk = line.split("=", 1)[1].strip('"')
                continue

            # key_mgmt
            if line.startswith("key_mgmt="):
                if not in_network_block:
                    raise RuntimeError(f"A setting for key management is found outside the network block.")

                key_mgmt = line.split("=", 1)[1]
                for item in KeyManagement:
                    if item.value == key_mgmt:
                        network.key_mgmt = item
                        break
                    else:
                        raise RuntimeError(f"Invalid key management setting.")
                continue

            # priority
            if line.startswith("priority="):
                if not in_network_block:
                    raise RuntimeError(f"A setting for network priority is found outside the network block.")

                network.priority = int(line.split("=", 1)[1])
                continue
