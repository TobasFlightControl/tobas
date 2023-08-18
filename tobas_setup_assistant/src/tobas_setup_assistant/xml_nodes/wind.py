import math
from xml.etree import ElementTree as ET


class WindModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        mean_wind_speed: float,
        const_wind_direction: float,
    ):
        assert mean_wind_speed >= 0.
        assert 0. <= const_wind_direction < 2 * math.pi

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_wind_plugin.so"
        plugin.attrib["name"] = f'{ns}_wind_plugin'

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "meanWindSpeed").text = str(mean_wind_speed)
        ET.SubElement(plugin, "constantWindDirection").text = str(const_wind_direction)
