from xml.etree import ElementTree as ET

from .sensor import SensorModel


class ImuModel(SensorModel):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        gyro_noise_density: float,
        gyro_random_walk: float,
        gyro_bias_corr_time: float,
        gyro_turn_on_bias_sigma: float,
        acc_noise_density: float,
        acc_random_walk: float,
        acc_bias_corr_time: float,
        acc_turn_on_bias_sigma: float,
    ) -> None:
        assert update_rate > 0.
        assert gyro_noise_density > 0.
        assert gyro_random_walk > 0.
        assert gyro_bias_corr_time > 0.
        assert gyro_turn_on_bias_sigma > 0.
        assert acc_noise_density > 0.
        assert acc_random_walk > 0.
        assert acc_bias_corr_time > 0.
        assert acc_turn_on_bias_sigma > 0.

        super().__init__(link_name, f'{ns}_imu', "imu", update_rate)

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(self.sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_imu_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_imu_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "imuTopic").text = "imu"
        ET.SubElement(plugin, "gyroscopeNoiseDensity").text = str(gyro_noise_density)
        ET.SubElement(plugin, "gyroscopeRandomWalk").text = str(gyro_random_walk)
        ET.SubElement(plugin, "gyroscopeBiasCorrelationTime").text = str(gyro_bias_corr_time)
        ET.SubElement(plugin, "gyroscopeTurnOnBiasSigma").text = str(gyro_turn_on_bias_sigma)
        ET.SubElement(plugin, "accelerometerNoiseDensity").text = str(acc_noise_density)
        ET.SubElement(plugin, "accelerometerRandomWalk").text = str(acc_random_walk)
        ET.SubElement(plugin, "accelerometerBiasCorrelationTime").text = str(acc_bias_corr_time)
        ET.SubElement(plugin, "accelerometerTurnOnBiasSigma").text = str(acc_turn_on_bias_sigma)
