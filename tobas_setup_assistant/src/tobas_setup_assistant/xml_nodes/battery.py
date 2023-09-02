from xml.etree import ElementTree as ET


class BatteryModel(ET.Element):
    SLOWDOWN_SIM = 10.0

    def __init__(
        self,
        ns: str,
        nominal_voltage: float,
    ) -> None:
        assert nominal_voltage > 0.0

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_battery_plugin.so"
        plugin.attrib["name"] = f"{ns}_battery_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "batteryPubTopic").text = "battery"
        ET.SubElement(plugin, "nominalVoltage").text = str(nominal_voltage)
