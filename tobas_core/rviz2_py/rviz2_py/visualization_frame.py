from PyQt5.QtWidgets import QWidget

from rviz2_py import rviz2


class VisualizationFrame(rviz2.VisualizationFrame):

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

    def setHelpPath(self, help_path: str) -> None:
        return super().setHelpPath(help_path)

    def setSplashPath(self, splash_path: str) -> None:
        return super().setSplashPath(splash_path)

    def initialize(self, display_config_file: str = "") -> None:
        return super().initialize(display_config_file)

    def setDisplayTitleFormat(self, title_format: str) -> None:
        return super().setDisplayTitleFormat(title_format)

    def getManager(self) -> QWidget:
        return super().getManager()

    def loadDisplayConfig(self, path: str) -> None:
        return super().loadDisplayConfig(path)

    def saveDisplayConfig(self, path: str) -> bool:
        return super().saveDisplayConfig(path)

    def getErrorMessage(self) -> str:
        return super().getErrorMessage()

    def setHideButtonVisibility(self, visible: bool) -> None:
        return super().setHideButtonVisibility(visible)
