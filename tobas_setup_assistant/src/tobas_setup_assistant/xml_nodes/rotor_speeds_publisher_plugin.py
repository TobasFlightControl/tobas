from xml.etree import ElementTree as ET
from typing import List

from .common import ListElement


class RotorSpeedsPublisherPlugin(ET.Element):
    def __init__(self, ns: str, rotor_joint_names: List[str]) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_rotor_speeds_publisher_plugin.so"
        plugin.attrib["name"] = f"{ns}_rotor_speeds_publisher_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        plugin.append(ListElement("rotorJointNames", rotor_joint_names))
