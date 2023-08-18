import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

TITLE = "Tobas Setup Assistant"
CONFIG_PATH = osp.join(osp.expanduser("~"), ".config/tobas_setup_assistant/config.ini")
DEFAULT = "DEFAULT"

# 汎用のポイントサイズ
TITLE_PSIZE = 18
LABEL_PSIZE = 12
BODY_PSIZE = 9


class Signals(QObject):
    
    airframe_updated = pyqtSignal()
