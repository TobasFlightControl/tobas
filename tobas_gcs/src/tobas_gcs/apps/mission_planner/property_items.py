from tobas_rqt_tools.widgets import SpinBox, DoubleSpinBox, ComboBox

from .structs import AltitudeFrame

LAT_LON_DECIMALS = 9


class LatitudeSpinBox(DoubleSpinBox):
    LABEL = "Latitude"

    def __init__(self) -> None:
        super().__init__()

        self.setMinimum(-90.0)
        self.setMaximum(+90.0)
        self.setDecimals(LAT_LON_DECIMALS)
        self.setSuffix(" deg")


class LongitudeSpinBox(DoubleSpinBox):
    LABEL = "Longitude"

    def __init__(self) -> None:
        super().__init__()

        self.setMinimum(-180.0)
        self.setMaximum(+180.0)
        self.setDecimals(LAT_LON_DECIMALS)
        self.setSuffix(" deg")


class AltitudeSpinBox(DoubleSpinBox):
    LABEL = "Altitude"

    def __init__(self) -> None:
        super().__init__()

        self.setValue(5.0)
        self.setDecimals(3)
        self.setSuffix(" m")


class AltitudeFrameComboBox(ComboBox):
    LABEL = "Altitude Frame"

    def __init__(self) -> None:
        super().__init__()

        self.addItems(AltitudeFrame.values())
        self.setCurrentText(AltitudeFrame.RELATIVE_TO_HOME.value)


class AcceptanceRadiusSpinBox(DoubleSpinBox):
    LABEL = "Acceptance Radius"

    def __init__(self) -> None:
        super().__init__()

        self.setMinimum(1e-3)
        self.setValue(1.0)
        self.setDecimals(3)
        self.setSuffix(" m")


class AltitudeToleranceSpinBox(DoubleSpinBox):
    LABEL = "Altitude Tolerance"

    def __init__(self) -> None:
        super().__init__()

        self.setMinimum(1e-3)
        self.setValue(0.1)
        self.setDecimals(3)
        self.setSuffix(" m")


class DurationSpinBox(SpinBox):
    LABEL = "Duration"

    def __init__(self) -> None:
        super().__init__()

        self.setMinimum(1)
        self.setValue(10)
        self.setSuffix(" s")
