from PyQt5.QtWidgets import QLabel


class FramedLabel(QLabel):
    def __init__(self, text: str = "") -> None:
        super().__init__(text=text)

        self.setStyleSheet("QLabel { border: 1px solid black; background-color: white; }")
