from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.utils import place_center

from .commands import Commands


class BasePropertyWidget(QWidget):
    NAME = "Unknown"

    LABEL_PSIZE = 12
    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    delete_button_clicked = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel(self.NAME)
        label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        place_center(label, rows)

        self._rows = QVBoxLayout()
        rows.addLayout(self._rows)

        rows.addStretch()

        self._delete_button = QPushButton("Delete Command")
        self._delete_button.setStyleSheet("background-color: red")
        self._delete_button.clicked.connect(self._on_delete_button_clicked)
        place_center(self._delete_button, rows)

    @pyqtSlot()
    def _on_delete_button_clicked(self) -> None:
        self.delete_button_clicked.emit()


class WaypointPropertyWidget(BasePropertyWidget):
    NAME = Commands.WAYPOINT.value

    def __init__(self) -> None:
        super().__init__()

        # TODO


class TakeoffPropertyWidget(BasePropertyWidget):
    NAME = Commands.TAKEOFF.value

    def __init__(self) -> None:
        super().__init__()

        # TODO


class LandPropertyWidget(BasePropertyWidget):
    NAME = Commands.LAND.value

    def __init__(self) -> None:
        super().__init__()

        # TODO


class RTHPropertyWidget(BasePropertyWidget):
    NAME = Commands.RETURN_TO_HOME.value

    def __init__(self) -> None:
        super().__init__()

        # TODO
