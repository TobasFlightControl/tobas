import os.path as osp
import rospkg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import MainWidget

from .common import *


class GroundControlStation(MainWidget):
    def __init__(self) -> None:
        super().__init__(PKG_NAME, DEFAULT)

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        self.signals = Signals()

        rows = QVBoxLayout()
        self.setLayout(rows)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        pass
