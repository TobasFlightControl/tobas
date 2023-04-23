import os
import re
import math
import rospy
from xml.etree import ElementTree as ET
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


def remap(x: float, a: float, b: float, c: float, d: float) -> float:
    """ xを[a, b]の範囲から[c, d]の範囲に投影する． """
    assert a <= b
    assert c <= d

    if a == b:
        return (c + d) / 2.
    else:
        return (c * (b - x) + d * (x - a)) / (b - a)


def get_user_name() -> str:
    """ ユーザ名を返す． """
    return os.environ['USER']


def get_drone_name() -> str:
    """ URDFからドローンの名前を取得する． """
    description = rospy.get_param("/robot_description")
    root = ET.fromstring(description)
    assert root.tag == "robot"
    return root.get("name")


def is_valid_email(email: str) -> bool:
    """
    Emailアドレスが有効かどうかを判定する．
    cf. https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
    """
    regex = r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b'
    return re.fullmatch(regex, email)


def rpm_to_rad_per_sec(rpm: float) -> float:
    return (math.pi / 30) * rpm


def add_expanding_widget(layout: QBoxLayout) -> None:
    """
    余白が空いているとサイズ固定が効かなくなるため，最後に伸縮可能なダミーウィジェットを加える．\\
    ダミーウィジェットを最大まで拡大するようにしておけば，他の要素はなるべく上に詰めてくれる．
    """
    dummy_widget = QWidget()
    dummy_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    layout.addWidget(dummy_widget)


def add_center_button(text: str, layout: QBoxLayout) -> QPushButton:
    """ layoutの中央にQPushButtonを配置する． """
    button = QPushButton(text)
    button_widget = QWidget()
    button_layout = QVBoxLayout()
    button_layout.setAlignment(Qt.AlignCenter)  # この操作のためにLayoutが必要
    button_widget.setLayout(button_layout)
    button_layout.addWidget(button)
    layout.addWidget(button_widget)  # この操作のためにWidgetが必要
    return button
