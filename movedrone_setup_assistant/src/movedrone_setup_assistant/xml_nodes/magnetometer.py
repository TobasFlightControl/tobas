from xml.etree import ElementTree as ET


class MagnetometerModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        ref_mag_north: float,
        ref_mag_east: float,
        ref_mag_down: float,
        gauss_noise: float,
        uniform_noise: float,
    ) -> None:
        assert gauss_noise > 0., f'gauss_noise = {gauss_noise}'
        assert uniform_noise > 0., f'uniform_noise = {uniform_noise}'

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libdh_gazebo_magnetometer_plugin.so"
        plugin.attrib["name"] = "dh_gazebo_magnetometer_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "magnetometerTopic").text = "magnetic_field"
        ET.SubElement(plugin, "refMagNorth").text = f'{ref_mag_north}'
        ET.SubElement(plugin, "refMagEast").text = f'{ref_mag_east}'
        ET.SubElement(plugin, "refMagDown").text = f'{ref_mag_down}'
        ET.SubElement(plugin, "noiseNormal").text = f'{gauss_noise} {gauss_noise} {gauss_noise}'
        ET.SubElement(plugin, "noiseUniformInitialBias").text \
            = f'{uniform_noise} {uniform_noise} {uniform_noise}'
