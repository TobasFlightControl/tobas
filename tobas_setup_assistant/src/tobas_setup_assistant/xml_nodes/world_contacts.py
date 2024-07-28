from xml.etree import ElementTree as ET


class WorldContactsModel(ET.Element):
    def __init__(self, ns: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_world_contacts_plugin.so"
        plugin.attrib["name"] = f"{ns}_world_contacts_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
