from xml.etree import ElementTree as ET

from .sensor import SensorModel


class MagnetometerModel(SensorModel):

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

        # 便宜的にセンサタイプをIMUにしている
        super().__init__(link_name, f'{ns}_magnetometer', "imu", update_rate)

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(self.sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_magnetometer_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_magnetometer_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "magnetometerTopic").text = "magnetic_field"
        ET.SubElement(plugin, "refMagNorth").text = str(ref_mag_north)
        ET.SubElement(plugin, "refMagEast").text = str(ref_mag_east)
        ET.SubElement(plugin, "refMagDown").text = str(ref_mag_down)
        ET.SubElement(plugin, "noiseNormal").text = " ".join([str(gauss_noise)] * 3)
        ET.SubElement(plugin, "noiseUniformInitialBias").text = " ".join([str(uniform_noise)] * 3)
