from typing import List
from dataclasses import dataclass
from enum import Enum


class KeyManagement(Enum):
    WPA_PSK = "WPA-PSK"  # 個人用WPA/WPA2
    WPA_EAP = "WPA-EAP"  # 企業用WPA/WPA2


class CountryCode(Enum):
    """Country Codes: https://countrycode.org/"""

    Afghanistan = "AF"
    Albania = "AL"
    Algeria = "DZ"
    American_Samoa = "AS"
    Andorra = "AD"
    Angola = "AO"
    Anguilla = "AI"
    Antarctica = "AQ"
    Antigua_and_Barbuda = "AG"
    Argentina = "AR"
    Armenia = "AM"
    Aruba = "AW"
    Australia = "AU"
    Austria = "AT"
    Azerbaijan = "AZ"
    Bahamas = "BS"
    Bahrain = "BH"
    Bangladesh = "BD"
    Barbados = "BB"
    Belarus = "BY"
    Belgium = "BE"
    Belize = "BZ"
    Benin = "BJ"
    Bermuda = "BM"
    Bhutan = "BT"
    Bolivia = "BO"
    Bosnia_and_Herzegovina = "BA"
    Botswana = "BW"
    Brazil = "BR"
    British_Indian_OceanTerritory = "IO"
    British_Virgin_Islands = "VG"
    Brunei = "BN"
    Bulgaria = "BG"
    Burkina_Faso = "BF"
    Burundi = "BI"
    Cambodia = "KH"
    Cameroon = "CM"
    Canad = "CA"
    Cape_Verde = "CV"
    Cayman_Islands = "KY"
    Central_African_Republic = "CF"
    Chad = "TD"
    Chile = "CL"
    China = "CN"
    Christmas_Island = "CX"
    Cocos_Islands = "CC"
    Colombia = "CO"
    Comoros = "KM"
    Cook_Islands = "CK"
    Costa_Rica = "CR"
    Croatia = "HR"
    Cuba = "CU"
    Curacao = "CW"
    Cyprus = "CY"
    Czech_Republic = "CZ"
    Democratic_Republic_of_the_Congo = "CD"
    Denmark = "DK"
    Djibouti = "DJ"
    Dominica = "DM"
    Dominican_Republic = "DO"
    East_Timor = "TL"
    Ecuador = "EC"
    Egypt = "EG"
    El_Salvador = "SV"
    Equatorial_Guinea = "GQ"
    Eritrea = "ER"
    Estonia = "EE"
    Ethiopia = "ET"
    Falkland_Islands = "FK"
    Faroe_Islands = "FO"
    Fiji = "FJ"
    Finland = "FI"
    France = "FR"
    French_Polynesia = "PF"
    Gabon = "GA"
    Gambia = "GM"
    Georgia = "GE"
    Germany = "DE"
    Ghana = "GH"
    Gibraltar = "GI"
    Greece = "GR"
    Greenland = "GL"
    Grenada = "GD"
    Guam = "GU"
    Guatemala = "GT"
    Guernsey = "GG"
    Guinea = "GN"
    Guinea_Bissau = "GW"
    Guyana = "GY"
    Haiti = "HT"
    Honduras = "HN"
    Hong_Kong = "HK"
    Hungary = "HU"
    Iceland = "IS"
    India = "IN"
    Indonesia = "ID"
    Iran = "IR"
    Iraq = "IQ"
    Ireland = "IE"
    Isle_of_Man = "IM"
    Israel = "IL"
    Italy = "IT"
    Ivory_Coast = "CI"
    Jamaica = "JM"
    Japan = "JP"
    Jersey = "JE"
    Jordan = "JO"
    Kazakhsta = "KZ"
    Kenya = "KE"
    Kiribati = "KI"
    Kosovo = "XK"
    Kuwait = "KW"
    Kyrgyzstan = "KG"
    Laos = "LA"
    Latvia = "LV"
    Lebanon = "LB"
    Lesotho = "LS"
    Liberia = "LR"
    Libya = "LY"
    Liechtenstein = "LI"
    Lithuania = "LT"
    Luxembourg = "LU"
    Macau = "MO"
    Macedonia = "MK"
    Madagascar = "MG"
    Malawi = "MW"
    Malaysia = "MY"
    Maldives = "MV"
    Mali = "ML"
    Malta = "MT"
    Marshall_Islands = "MH"
    Mauritania = "MR"
    Mauritius = "MU"
    Mayotte = "YT"
    Mexico = "MX"
    Micronesia = "FM"
    Moldova = "MD"
    Monaco = "MC"
    Mongolia = "MN"
    Montenegro = "ME"
    Montserrat = "MS"
    Morocco = "MA"
    Mozambique = "MZ"
    Myanmar = "MM"
    Namibia = "NA"
    Nauru = "NR"
    Nepal = "NP"
    Netherlands = "NL"
    Netherlands_Antilles = "AN"
    New_Caledonia = "NC"
    New_Zealand = "NZ"
    Nicaragua = "NI"
    Niger = "NE"
    Nigeria = "NG"
    Niue = "NU"
    North_Korea = "KP"
    Northern_Mariana_Islands = "MP"
    Norway = "NO"
    Oman = "OM"
    Pakistan = "PK"
    Palau = "PW"
    Palestine = "PS"
    Panama = "PA"
    Papua_New_Guinea = "PG"
    Paraguay = "PY"
    Peru = "PE"
    Philippines = "PH"
    Pitcairn = "PN"
    Poland = "PL"
    Portugal = "PT"
    Puerto_Rico = "PR"
    Qatar = "QA"
    Republic_of_the_Congo = "CG"
    Reunion = "RE"
    Romania = "RO"
    Russi = "RU"
    Rwanda = "RW"
    Saint_Barthelemy = "BL"
    Saint_Helena = "SH"
    Saint_Kitts_and_Nevis = "KN"
    Saint_Lucia = "LC"
    Saint_Martin = "MF"
    Saint_Pierre_and_Miquelon = "PM"
    Saint_Vincent_and_the_Grenadines = "VC"
    Samoa = "WS"
    San_Marino = "SM"
    Sao_Tome_and_Principe = "ST"
    Saudi_Arabia = "SA"
    Senegal = "SN"
    Serbia = "RS"
    Seychelles = "SC"
    Sierra_Leone = "SL"
    Singapore = "SG"
    Sint_Maarten = "SX"
    Slovakia = "SK"
    Slovenia = "SI"
    Solomon_Islands = "SB"
    Somalia = "SO"
    South_Africa = "ZA"
    South_Korea = "KR"
    South_Sudan = "SS"
    Spain = "ES"
    Sri_Lanka = "LK"
    Sudan = "SD"
    Suriname = "SR"
    Svalbard_and_Jan_Mayen = "SJ"
    Swaziland = "SZ"
    Sweden = "SE"
    Switzerland = "CH"
    Syria = "SY"
    Taiwan = "TW"
    Tajikistan = "TJ"
    Tanzania = "TZ"
    Thailand = "TH"
    Togo = "TG"
    Tokelau = "TK"
    Tonga = "TO"
    Trinidad_and_Tobago = "TT"
    Tunisia = "TN"
    Turkey = "TR"
    Turkmenistan = "TM"
    Turks_and_Caicos_Islands = "TC"
    Tuvalu = "TV"
    US_Virgin_Islands = "VI"
    Uganda = "UG"
    Ukraine = "UA"
    United_Arab_Emirates = "AE"
    United_Kingdom = "GB"
    United_State = "US"
    Uruguay = "UY"
    Uzbekistan = "UZ"
    Vanuatu = "VU"
    Vatican = "VA"
    Venezuela = "VE"
    Vietnam = "VN"
    Wallis_and_Futuna = "WF"
    Western_Sahara = "EH"
    Yemen = "YE"
    Zambia = "ZM"
    Zimbabwe = "ZW"


@dataclass
class Network:
    ssid: str = ""
    psk: str = ""
    key_mgmt: KeyManagement = KeyManagement.WPA_PSK
    priority: int = 0


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
