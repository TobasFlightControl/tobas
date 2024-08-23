import markdown
from PyQt5.QtWidgets import QTextBrowser


class MarkDownWidget(QTextBrowser):
    """マークダウン形式のテキストを表示するウィジェット．数式は書けない．"""

    def __init__(self) -> None:
        super().__init__()

        # 背景色を透明にし，枠線を消す設定
        # これで親ウィジェットに自然にマークダウンテキストを組み込める
        self.setStyleSheet("background-color: transparent; border: none")

    def set_text(self, markdown_text: str) -> None:
        html = markdown.markdown(markdown_text)
        self.setHtml(html)
