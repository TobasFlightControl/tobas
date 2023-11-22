from xml.etree import ElementTree as ET


class BatteryModel(ET.Element):
    def __init__(
        self,
        ns: str,
        max_voltage: float,
        sag_voltage: float,
        max_current: float,
        capacity: float,
        num_rotors: int,
    ) -> None:
        assert max_voltage > 0.0
        assert sag_voltage > 0.0
        assert max_current > 0.0
        assert capacity > 0.0
        assert num_rotors >= 0

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_battery_plugin.so"
        plugin.attrib["name"] = f"{ns}_battery_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "maxVoltage").text = str(max_voltage)
        ET.SubElement(plugin, "sagVoltage").text = str(sag_voltage)
        ET.SubElement(plugin, "maxCurrent").text = str(max_current)
        ET.SubElement(plugin, "currentCapacity").text = str(capacity)
        ET.SubElement(plugin, "numRotors").text = str(num_rotors)
