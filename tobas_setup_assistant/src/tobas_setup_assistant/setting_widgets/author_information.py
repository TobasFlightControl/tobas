from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from overrides import overrides
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_spacer
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..utils import is_valid_email, get_user_name, get_git_user_email


class AuthorInformationWidget(BaseSettingWidget):
    NAME = "Author Info"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Specify Author Information"
        abst_text = (
            "生成されるパッケージの管理者に関する情報を入力してください．" + "ここで指定した情報はパッケージのpackage.xmlに反映されます．"
        )
        super().__init__(main, title_text, abst_text)

        self.name = ParamGetterWidget_LineEdit(
            "Name of the Maintainer",
            default=get_user_name(),
        )
        self._rows.addWidget(self.name)

        self.email = ParamGetterWidget_LineEdit(
            "Email of the Maintainer",
            default=get_git_user_email(),
        )
        self._rows.addWidget(self.email)

        add_spacer(self._rows)

    @overrides
    def define_connections(self) -> None:
        super().define_connections()

    @overrides
    def is_valid(self) -> bool:
        author_name = self.name.get()
        if author_name == "":
            q_error_named(self._main, self.NAME, "Author name is blank.")
            return False

        author_email = self.email.get()
        if not is_valid_email(author_email):
            q_error_named(self._main, self.NAME, "Invalid email address.")
            return False

        return True
