from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from PyQt5.QtWidgets import QButtonGroup, QCheckBox, QStackedWidget

from ..base_setting import BaseSettingWidget
from .urdf_loader import URDFLoaderWidget
from .package_loader import PackageLoaderWidget


class StartWidget(BaseSettingWidget):
    NAME = "Start"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Tobas Setup Assistant"
        abst_text = (
            "The Tobas Setup Assistant is a GUI tool designed for creating configuration files "
            "needed to operate drones with Tobas. "
            "It utilizes the URDF created in the previous steps and allows for the configuration of elements "
            "not expressed in the URDF, "
            "such as propeller aerodynamics and controller settings."
        )
        super().__init__(main, title_text, abst_text)

        ckb_group = QButtonGroup(parent=self)  # コンストラクタで解放されないように親ウィジェットを設定
        stacked_widget = QStackedWidget()

        new_ckb = QCheckBox("Create new Tobas configuration package")
        self._urdf_loader = URDFLoaderWidget(main)
        ckb_group.addButton(new_ckb)
        ckb_group.setId(new_ckb, 0)
        stacked_widget.addWidget(self._urdf_loader)

        edit_ckb = QCheckBox("Edit existing Tobas configuration package")
        self._pkg_loader = PackageLoaderWidget(main)
        ckb_group.addButton(edit_ckb)
        ckb_group.setId(edit_ckb, 1)
        stacked_widget.addWidget(self._pkg_loader)

        new_ckb.setChecked(True)  # デフォルト
        ckb_group.setExclusive(True)  # 1つのみ有効
        ckb_group.buttonClicked[int].connect(stacked_widget.setCurrentIndex)  # ボタンに合わせて表示する内容を変更

        # レイアウト
        self._rows.addWidget(new_ckb)
        self._rows.addWidget(edit_ckb)
        self._rows.addWidget(stacked_widget)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        return True
