from xml.etree import ElementTree as ET


class BarometerModel(ET.Element):

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

        # robot/gazebo
        super().__init__("gazebo", reference=link_name)

        # robot/gazebo/sensor
        sensor = ET.SubElement(self, "sensor")
        sensor.attrib["name"] = f'{ns}_barometer'

        # SDFormat(http://sdformat.org/spec?elem=sensor)にはあるが，"air_pressure"だとセンサが起動しなかった
        # <air_pressure>タグを指定しておらずtypeは正直何でもよいため，とりあえず"imu"にしている
        # "altimeter"もあるが"Error: Conversion of sensor type[altimeter] not supported"が出る
        sensor.attrib["type"] = "imu"

        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = f'{update_rate}'
        ET.SubElement(sensor, "visualize").text = "false"
        ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_barometer_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_barometer_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "pressureTopic").text = "air_pressure"
        ET.SubElement(plugin, "altitudeZero").text = f'{altitude_0}'
        ET.SubElement(plugin, "pressureVariance").text = f'{pressure_var}'
