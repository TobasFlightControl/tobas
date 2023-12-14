import os.path as osp

# ConfigParser
CONFIG_PATH = osp.join(
    osp.expanduser("~"), ".config/ardupilot_dynamic_configuration/config.ini"
)
DEFAULT = "DEFAULT"

TITLE = "ArduPilot Dynamic Configuration"
FLOAT_DECIMALS = 3
WAIT_FOR_SERVICE = 0.1

PARAM_SET_SRV_NAME = "mavros/param/set"
