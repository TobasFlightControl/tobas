import rospy
from xml.etree import ElementTree as ET


def get_drone_name() -> str:
    """URDFからドローンの名前を取得する．"""
    description = rospy.get_param("/robot_description")
    root = ET.fromstring(description)
    assert root.tag == "robot"

    name = root.get("name")
    return name if name else "unknown"
