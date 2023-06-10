from xml.etree import ElementTree as ET

from .sensor import SensorModel


class BarometerModel(SensorModel):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        altitude_0: float,
        pressure_var: float,
    ) -> None:
        assert update_rate > 0.
        assert altitude_0 >= 0.
        assert pressure_var > 0.

        # 便宜的にセンサタイプをIMUにしている
        super().__init__(link_name, f'{ns}_barometer', "imu", update_rate)

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(self.sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_barometer_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_barometer_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "pressureTopic").text = "air_pressure"
        ET.SubElement(plugin, "altitudeZero").text = str(altitude_0)
        ET.SubElement(plugin, "pressureVariance").text = str(pressure_var)
