from dataclasses import dataclass
from enum import Enum


class KeyManagement(Enum):
    WPA_PSK = "WPA-PSK"  # 個人用WPA/WPA2
    WPA_EAP = "WPA-EAP"  # 企業用WPA/WPA2


@dataclass
class Network:
    ssid: str = ""
    psk: str = ""
    key_mgmt: KeyManagement = KeyManagement.WPA_PSK
    priority: int = 0
