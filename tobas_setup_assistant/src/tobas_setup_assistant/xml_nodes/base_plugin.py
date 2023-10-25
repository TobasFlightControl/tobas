from xml.etree import ElementTree as ET
from typing import List

from .common import ListElement


class BasePlugin(ET.Element):
    def __init__(self, ns: str, rotor_joint_names: List[str]) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_base_plugin.so"
        plugin.attrib["name"] = f"{ns}_base_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        plugin.append(ListElement("rotorJointNames", rotor_joint_names))
