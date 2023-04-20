from xml.etree import ElementTree as ET


class MagnetometerModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        ref_mag_north: float,
        ref_mag_east: float,
        ref_mag_down: float,
        gauss_noise: float,
        uniform_noise: float,
    ) -> None:
        assert update_rate > 0.
        assert gauss_noise > 0., f'gauss_noise = {gauss_noise}'
        assert uniform_noise > 0., f'uniform_noise = {uniform_noise}'

        # robot/gazebo
        super().__init__("gazebo", reference=link_name)

        # robot/gazebo/sensor
        sensor = ET.SubElement(self, "sensor")
        sensor.attrib["name"] = f'{ns}_magnetometer'
        sensor.attrib["type"] = "magnetometer"

        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = f'{update_rate}'
        ET.SubElement(sensor, "visualize").text = "false"
        ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_magnetometer_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_magnetometer_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "magnetometerTopic").text = "magnetic_field"
        ET.SubElement(plugin, "refMagNorth").text = f'{ref_mag_north}'
        ET.SubElement(plugin, "refMagEast").text = f'{ref_mag_east}'
        ET.SubElement(plugin, "refMagDown").text = f'{ref_mag_down}'
        ET.SubElement(plugin, "noiseNormal").text = f'{gauss_noise} {gauss_noise} {gauss_noise}'
        ET.SubElement(plugin, "noiseUniformInitialBias").text \
            = f'{uniform_noise} {uniform_noise} {uniform_noise}'
