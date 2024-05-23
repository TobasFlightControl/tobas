from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont

PKG_NAME = "tobas_gcs"
TITLE = "Tobas"
SOURCE_CMD = "source /opt/ros/noetic/setup.bash && source /opt/tobas/setup.bash"

TO_DO = "TODO"
CONFIG_PKG_NOT_LOADED = "Tobas configuration package is not loaded yet."
NOT_IMPLEMENTED = "Not implemented yet."

TITLE_PSIZE = 18
LABEL_PSIZE = 12
BODY_PSIZE = 9
WAIT_FOR_SERVER = 10.0  # [s] ラズパイ側のサーバに接続する際のタイムアウト
PAINT_REFRESH_PERIOD = 100  # [ms] ペイントの更新周期．短すぎるとフリーズの恐れあり．

# Raspberry Pi
CATKIN_WS_TOBAS = "/etc/tobas/catkin_ws/"  # Tobasパッケージ用ワークスペース


class Description(QLabel):
    def __init__(self, text: str) -> None:
        super().__init__(text)

        self.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.setAlignment(Qt.AlignTop)
        self.setWordWrap(True)
        self.setOpenExternalLinks(True)
