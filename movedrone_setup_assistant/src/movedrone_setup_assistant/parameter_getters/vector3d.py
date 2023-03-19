from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from typing import List, Tuple

from dh_rqt_tools.widgets import DoubleSpinBox

from .base import ParamGetterWidget
from ..const import *


class ParamGetterWidget_Vector3d(ParamGetterWidget):

    value_changed = pyqtSignal(float, float, float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        minimum: List[float] = [-1e+9] * 3,
        maximum: List[float] = [+1e+9] * 3,
        single_step: List[float] = [1.] * 3,
        default: List[float] = [0.] * 3,
        suffix: str = "",
    ) -> None:
        assert len(minimum) == 3
        assert len(maximum) == 3
        assert len(single_step) == 3
        assert len(default) == 3

        super().__init__(param_name, description_text)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._x = DoubleGetter("x", minimum[0], maximum[0], single_step[0], default[0], suffix)
        self._cols.addWidget(self._x)

        self._y = DoubleGetter("y", minimum[1], maximum[1], single_step[1], default[1], suffix)
        self._cols.addWidget(self._y)

        self._z = DoubleGetter("z", minimum[2], maximum[2], single_step[2], default[2], suffix)
        self._cols.addWidget(self._z)

        self._x.data.valueChanged.connect(self._on_value_changed)
        self._y.data.valueChanged.connect(self._on_value_changed)
        self._z.data.valueChanged.connect(self._on_value_changed)

    def get(self) -> Tuple[float, float, float]:
        return self._x.get(), self._y.get(), self._z.get()

    def set(self, x: float, y: float, z: float) -> None:
        self._x.data.setValue(x)
        self._y.data.setValue(y)
        self._z.data.setValue(z)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self._x.get(), self._y.get(), self._z.get())


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
        label.setAlignment(Qt.AlignRight)
        self._cols.addWidget(label)

        self.data = DoubleSpinBox()
        self.data.setMinimum(minimum)
        self.data.setMaximum(maximum)
        self.data.setSingleStep(single_step)
        if default is not None:
            assert minimum <= default <= maximum
            self.data.setValue(default)
        self.data.setSuffix(suffix)
        self._cols.addWidget(self.data)

    def get(self) -> float:
        return self.data.value()
