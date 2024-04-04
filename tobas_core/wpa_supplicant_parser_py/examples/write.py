import os.path as osp

from wpa_supplicant_parser_py.parser import WPASupplicantParser

if __name__ == "__main__":
    parser = WPASupplicantParser()

    conf_path = osp.join(osp.dirname(__file__), "wpa_supplicant.conf")
    parser.parse_from_file(conf_path)

    parser.write("./wpa_supplicant_copy.conf")
