from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant
    from ..parameter_getters import ParamGetterWidget

import os
from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from tobas_std_tools_py.string import is_valid_email
from tobas_std_tools_py.git import get_git_user_email
from tobas_rqt_tools.messages import q_error_named

from ..parameter_getters import ParamGetterWidget_LineEdit
from .base_setting import BaseSettingWidget


class AuthorInformationWidget(BaseSettingWidget):
    NAME = "Author Info"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Specify Author Information"
        abst_text = (
            "Enter the name and email address of the person administering the Tobas package "
            "that you're creating with the Setup Assistant. "
            "This step is important for keeping track of package ownership and for any necessary future communications."
        )
        super().__init__(main, title_text, abst_text)

        self._param_rows = QVBoxLayout()
        self._rows.addLayout(self._param_rows)

        self.name = ParamGetterWidget_LineEdit("Name of the Maintainer", default=os.environ["USER"])
        self._param_rows.addWidget(self.name)

        self.email = ParamGetterWidget_LineEdit("Email of the Maintainer", default=get_git_user_email())
        self._param_rows.addWidget(self.email)

        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
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

    @override
    def dump_settings(self) -> dict:
        res = dict()
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()
        return res

    @override
    def load_settings(self, data: dict) -> None:
        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])
