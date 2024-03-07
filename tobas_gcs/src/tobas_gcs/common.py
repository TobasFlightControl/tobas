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

# Raspberry Pi
HOST_NAME = "navio.local"  # ラズパイのホスト名
PORT = 22  # SSHポート番号
USER = "pi"  # ユーザ名
LOGIN_PASSWORD = "raspberry"  # ログインパスワード
SUDO_PREFIX = f"echo {LOGIN_PASSWORD} | sudo -S "
CATKIN_WS_TOBAS = "/home/pi/.catkin_ws_tobas/"  # Tobasパッケージ用ワークスペース


class Signals(QObject):
    config_pkg_loaded = pyqtSignal()


class Description(QLabel):
    def __init__(self, text: str) -> None:
        super().__init__(text)

        self.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.setAlignment(Qt.AlignTop)
        self.setWordWrap(True)
        self.setOpenExternalLinks(True)
