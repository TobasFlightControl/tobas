from xml.etree import ElementTree as ET


class BarometerModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        ref_altitude: float,
        pressure_var: float,
    ) -> None:
        assert update_rate > 0.
        assert pressure_var > 0.

        # robot/gazebo
        super().__init__("gazebo", reference=link_name)

        # robot/gazebo/sensor
        sensor = ET.SubElement(self, "sensor")
        sensor.attrib["name"] = f'{ns}_barometer'

        # SDFormat(http://sdformat.org/spec?elem=sensor)にはあるが，"air_pressure"だとセンサが起動しなかった
        # <air_pressure>タグを指定しておらずtypeは正直何でもよいため，とりあえず"altimeter"にしている
        # sensor.attrib["type"] = "air_pressure"
        sensor.attrib["type"] = "altimeter"

        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = f'{update_rate}'
        ET.SubElement(sensor, "visualize").text = "false"
        ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(sensor, "plugin")
        plugin.attrib["filename"] = "libdh_gazebo_pressure_plugin.so"
        plugin.attrib["name"] = "dh_gazebo_pressure_sensor_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "pressureTopic").text = "air_pressure"
        ET.SubElement(plugin, "referenceAltitude").text = f'{ref_altitude}'
        ET.SubElement(plugin, "pressureVariance").text = f'{pressure_var}'
