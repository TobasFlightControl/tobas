from PyQt5.QtWidgets import QWidget, QLabel


class FramedLabel(QLabel):

    def __init__(self, parent: QWidget = None, text: str = "") -> None:
        super().__init__(parent=parent, text=text)

        self.setStyleSheet("QLabel { border: 1px solid black; background-color: white; }")
