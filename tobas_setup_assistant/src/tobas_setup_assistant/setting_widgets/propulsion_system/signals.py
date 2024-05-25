from PyQt5.QtCore import QObject, pyqtSignal


class PropulsionSystemSignals(QObject):
    add_link = pyqtSignal(str)  # selectedにリンクを追加
    remove_link = pyqtSignal(str)  # selectedからリンクを削除
