from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import DoubleSpinBox

from ..constants import *


class DoubleGetter(QWidget):

    def __init__(
        self,
        name: str,
        minimum: float,
        maximum: float,
        single_step: float,
        default: float,
        suffix: str,
    ) -> None:
        assert minimum < maximum
        assert single_step > 0.

        super().__init__()

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        label = QLabel(name + ":")
        label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._cols.addWidget(label)

        self.data = DoubleSpinBox()
        self.data.setMinimum(minimum)
        self.data.setMaximum(maximum)
        self.data.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self.data.setValue(default)
        self.data.setSuffix(suffix)
        self.data.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._cols.addWidget(self.data)

    def get(self) -> float:
        return self.data.value()
