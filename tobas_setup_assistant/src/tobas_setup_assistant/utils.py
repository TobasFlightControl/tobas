import os
import re
import math
import subprocess
import rospy
from xml.etree import ElementTree as ET
from typing import Sequence
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


def get_user_name() -> str:
    """PCのユーザ名を返す．"""
    return os.environ["USER"]


def get_git_user_name() -> str:
    """Gitのユーザ名を返す．"""
    command = "git config --global user.name"
    result = subprocess.run(command.split(), stdout=subprocess.PIPE)
    return result.stdout.decode("utf-8").strip()


def get_git_user_email() -> str:
    """Gitのメールアドレスを返す．"""
    command = "git config --global user.email"
    result = subprocess.run(command.split(), stdout=subprocess.PIPE)
    return result.stdout.decode("utf-8").strip()


def get_drone_name() -> str:
    """URDFからドローンの名前を取得する．"""
    description = rospy.get_param("/robot_description")
    root = ET.fromstring(description)
    assert root.tag == "robot"

    name = root.get("name")
    return name if name else "unknown"


def is_valid_email(email: str) -> bool:
    """
    Emailアドレスが有効かどうかを判定する．
    cf. https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
    """
    regex = r"\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b"
    return re.fullmatch(regex, email)


def all_gt(seq: Sequence[float], x: float) -> bool:
    for elem in seq:
        if elem <= x:
            return False
    return True


def all_lt(seq: Sequence[float], x: float) -> bool:
    for elem in seq:
        if elem >= x:
            return False
    return True


def all_ge(seq: Sequence[float], x: float) -> bool:
    for elem in seq:
        if elem < x:
            return False
    return True


def all_le(seq: Sequence[float], x: float) -> bool:
    for elem in seq:
        if elem > x:
            return False
    return True


def is_unique(lst: list):
    return len(lst) == len(set(lst))


def convert_superscript(text: str):
    """
    Converts digits following a caret (^) into their superscript equivalent.
    """
    superscript_map = {
        "0": "⁰",
        "1": "¹",
        "2": "²",
        "3": "³",
        "4": "⁴",
        "5": "⁵",
        "6": "⁶",
        "7": "⁷",
        "8": "⁸",
        "9": "⁹",
    }

    # Function to replace each match
    def replace_with_superscript(match: re.Match):
        return "".join(superscript_map[char] for char in match.group(1))

    # Replace all occurrences of ^ followed by one or more digits
    return re.sub(r"\^(\d+)", replace_with_superscript, text)
