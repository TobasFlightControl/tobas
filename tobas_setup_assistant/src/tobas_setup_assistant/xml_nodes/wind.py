from xml.etree import ElementTree as ET


class WindModel(ET.Element):
    def __init__(self, ns: str, link_name: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_wind_plugin.so"
        plugin.attrib["name"] = f"{ns}_wind_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
