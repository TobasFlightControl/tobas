from xml.etree import ElementTree as ET


class GroundTruthStateModel(ET.Element):
    def __init__(self, ns: str, link_name: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_ground_truth_state_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_ground_truth_state_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
