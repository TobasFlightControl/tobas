from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import SpinBox, DoubleSpinBox

from ..constants import *


class IntGetter(QWidget):

    def __init__(
        self,
        name: str,
        minimum: int,
        maximum: int,
        single_step: int,
        default: int,
        suffix: str,
    ) -> None:
        assert minimum <= maximum
        assert single_step > 0

        super().__init__()

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        label = QLabel(name + ":")
        label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._cols.addWidget(label)

        self.data = SpinBox()
        self.data.setMinimum(minimum)
        self.data.setMaximum(maximum)
        self.data.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self.data.setValue(default)
        self.data.setSuffix(suffix)
        self.data.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self._cols.addWidget(self.data)

    def get(self) -> int:
        return self.data.value()


class DoubleGetter(QWidget):

    def __init__(
        self,
        name: str,
        decimals: int,
        minimum: float,
        maximum: float,
        single_step: float,
        default: float,
        suffix: str,
    ) -> None:
        assert decimals >= 0
        assert minimum < maximum
        assert single_step > 0.

        super().__init__()

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        label = QLabel(name + ":")
        label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._cols.addWidget(label)

        self.data = DoubleSpinBox()
        self.data.setDecimals(decimals)
        self.data.setMinimum(minimum)
        self.data.setMaximum(maximum)
        self.data.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self.data.setValue(default)
        self.data.setSuffix(suffix)
        self.data.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self._cols.addWidget(self.data)

    def get(self) -> float:
        return self.data.value()
