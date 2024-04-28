from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.layouts import FormLayout
from tobas_rqt_tools.utils import place_center

from .structs import Commands
from .property_items import *


class BasePropertyWidget(QWidget):
    NAME = "Unknown"

    LABEL_PSIZE = 12
    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    value_changed = pyqtSignal()
    delete_button_clicked = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel(self.NAME)
        label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        place_center(label, rows)

        rows.addSpacing(30)

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

    def _emit_value_changed(self) -> None:
        self.value_changed.emit()


class WaypointPropertyWidget(BasePropertyWidget):
    NAME = Commands.WAYPOINT.value

    def __init__(self) -> None:
        super().__init__()

        form = FormLayout()
        self._rows.addLayout(form)

        self.latitude = LatitudeSpinBox()
        self.latitude.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(LatitudeSpinBox.LABEL), self.latitude)

        self.longitude = LongitudeSpinBox()
        self.longitude.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(LongitudeSpinBox.LABEL), self.longitude)

        self.altitude = AltitudeSpinBox()
        self.altitude.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(AltitudeSpinBox.LABEL), self.altitude)

        self.altitude_frame = AltitudeFrameComboBox()
        form.addRow(QLabel(AltitudeFrameComboBox.LABEL), self.altitude_frame)

        self.acceptance_radius = AcceptanceRadiusSpinBox()
        self.acceptance_radius.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(AcceptanceRadiusSpinBox.LABEL), self.acceptance_radius)

        self.duration = DurationSpinBox()
        self.duration.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(DurationSpinBox.LABEL), self.duration)


class TakeoffPropertyWidget(BasePropertyWidget):
    NAME = Commands.TAKEOFF.value

    def __init__(self) -> None:
        super().__init__()

        form = FormLayout()
        self._rows.addLayout(form)

        self.altitude = AltitudeSpinBox()
        self.altitude.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(AltitudeSpinBox.LABEL), self.altitude)

        self.altitude_frame = AltitudeFrameComboBox()
        form.addRow(QLabel(AltitudeFrameComboBox.LABEL), self.altitude_frame)

        self.duration = DurationSpinBox()
        self.duration.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(DurationSpinBox.LABEL), self.duration)


class LandPropertyWidget(BasePropertyWidget):
    NAME = Commands.LAND.value

    def __init__(self) -> None:
        super().__init__()

        form = FormLayout()
        self._rows.addLayout(form)

        self.duration = DurationSpinBox()
        self.duration.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(DurationSpinBox.LABEL), self.duration)


class RTHPropertyWidget(BasePropertyWidget):
    NAME = Commands.RETURN_TO_HOME.value

    def __init__(self) -> None:
        super().__init__()

        form = FormLayout()
        self._rows.addLayout(form)

        self.altitude = AltitudeSpinBox()
        self.altitude.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(AltitudeSpinBox.LABEL), self.altitude)

        self.altitude_frame = AltitudeFrameComboBox()
        form.addRow(QLabel(AltitudeFrameComboBox.LABEL), self.altitude_frame)

        self.acceptance_radius = AcceptanceRadiusSpinBox()
        self.acceptance_radius.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(AcceptanceRadiusSpinBox.LABEL), self.acceptance_radius)

        self.duration = DurationSpinBox()
        self.duration.valueChanged.connect(self._emit_value_changed)
        form.addRow(QLabel(DurationSpinBox.LABEL), self.duration)
