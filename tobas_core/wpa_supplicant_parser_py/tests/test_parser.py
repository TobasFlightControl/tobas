from wpa_supplicant_parser_py.parser import WPASupplicantParser

if __name__ == "__main__":
    config_text = """
country=JP
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

# Home
network={
    ssid="xg100n-96a421-1"
    psk="01dc4b0dcadd1"
    key_mgmt=WPA-PSK
    priority=0
}

# Aterm MR05LN                                    
network={
    ssid="aterm-875440"
    psk="11a0940898206"
    key_mgmt=WPA-PSK
    priority=1
}
"""

    parser = WPASupplicantParser()
    parser.parse_from_text(config_text)

    print(parser)
