import os
import re
import math
import subprocess
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
    """ PCのユーザ名を返す． """
    return os.environ['USER']


def get_git_user_name() -> str:
    """ Gitのユーザ名を返す． """
    command = "git config --global user.name"
    result = subprocess.run(command.split(), stdout=subprocess.PIPE)
    return result.stdout.decode('utf-8').strip()


def get_git_user_email() -> str:
    """ Gitのメールアドレスを返す． """
    command = "git config --global user.email"
    result = subprocess.run(command.split(), stdout=subprocess.PIPE)
    return result.stdout.decode('utf-8').strip()


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
