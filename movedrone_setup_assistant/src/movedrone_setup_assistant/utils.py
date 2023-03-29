import re
import math
import rospy
from xml.etree import ElementTree as ET
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


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
