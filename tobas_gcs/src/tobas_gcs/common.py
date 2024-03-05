import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


PKG_NAME = "tobas_gcs"
CONFIG_PATH = osp.expanduser(f"~/.config/{PKG_NAME}/config.ini")
DEFAULT = "DEFAULT"
TITLE = "Tobas"
UNKNOWN = "Unknown"

# Point Sizes
TITLE_PSIZE = 18
LABEL_PSIZE = 12
BODY_PSIZE = 9


class Signals(QObject):
    config_pkg_loaded = pyqtSignal()


class Description(QLabel):
    def __init__(self, text: str) -> None:
        super().__init__(text)

        self.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.setAlignment(Qt.AlignTop)
        self.setWordWrap(True)
        self.setOpenExternalLinks(True)
