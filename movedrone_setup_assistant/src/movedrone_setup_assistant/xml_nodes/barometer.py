from xml.etree import ElementTree as ET


class BarometerModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        ref_altitude: float,
        pressure_var: float,
    ) -> None:
        assert pressure_var > 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libdh_gazebo_pressure_plugin.so"
        plugin.attrib["name"] = "dh_gazebo_pressure_sensor_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "pressureTopic").text = "air_pressure"
        ET.SubElement(plugin, "referenceAltitude").text = f'{ref_altitude}'
        ET.SubElement(plugin, "pressureVariance").text = f'{pressure_var}'
