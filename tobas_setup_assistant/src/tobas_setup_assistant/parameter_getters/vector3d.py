from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from typing import Tuple, List

from .base import ParamGetterWidget
from .utils import DoubleGetter


class ParamGetterWidget_Vector3d(ParamGetterWidget):

    value_changed = pyqtSignal(float, float, float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        decimals: int = 3,
        minimum: Tuple[float, float, float] = (-1e+9,) * 3,
        maximum: Tuple[float, float, float] = (+1e+9,) * 3,
        single_step: Tuple[float, float, float] = (1.,) * 3,
        default: Tuple[float, float, float] = (0.,) * 3,
        suffix: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._x = DoubleGetter(
            "x", decimals, minimum[0], maximum[0], single_step[0], default[0], suffix
        )
        self._cols.addWidget(self._x)

        self._y = DoubleGetter(
            "y", decimals, minimum[1], maximum[1], single_step[1], default[1], suffix
        )
        self._cols.addWidget(self._y)

        self._z = DoubleGetter(
            "z", decimals, minimum[2], maximum[2], single_step[2], default[2], suffix
        )
        self._cols.addWidget(self._z)

        self._x.data.valueChanged.connect(self._on_value_changed)
        self._y.data.valueChanged.connect(self._on_value_changed)
        self._z.data.valueChanged.connect(self._on_value_changed)

    def x(self) -> float:
        return self._x.get()

    def y(self) -> float:
        return self._y.get()

    def z(self) -> float:
        return self._z.get()

    def get(self) -> List[float]:
        """ yamlにそのまま書き込めるようにタプルではなくリストで返す． """
        return [self.x(), self.y(), self.z()]

    def set(self, x: float, y: float, z: float) -> None:
        self._x.data.setValue(x)
        self._y.data.setValue(y)
        self._z.data.setValue(z)

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self.x(), self.y(), self.z())
