import rclpy
from overrides import override
from PyQt5.QtWidgets import QWidget


class Widget(QWidget):
    """
    ===== QWidgetとの違い =====
    - closeで子ウィジェットのcloseを再帰的に呼び出す
    """

    @override
    def close(self) -> bool:
        rclpy.logdebug(f"{self.__class__.__name__}.close")

        for child in self.findChildren(Widget):
            child.close()

        return super().close()
