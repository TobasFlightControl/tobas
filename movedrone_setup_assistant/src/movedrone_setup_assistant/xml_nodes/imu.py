from xml.etree import ElementTree as ET


class ImuModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        gyro_noise_density: float,
        gyro_random_walk: float,
        gyro_bias_corr_time: float,
        gyro_turn_on_bias_sigma: float,
        acc_noise_density: float,
        acc_random_walk: float,
        acc_bias_corr_time: float,
        acc_turn_on_bias_sigma: float,
    ) -> None:
        assert gyro_noise_density > 0.
        assert gyro_random_walk > 0.
        assert gyro_bias_corr_time > 0.
        assert gyro_turn_on_bias_sigma > 0.
        assert acc_noise_density > 0.
        assert acc_random_walk > 0.
        assert acc_bias_corr_time > 0.
        assert acc_turn_on_bias_sigma > 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libdh_gazebo_imu_plugin.so"
        plugin.attrib["name"] = "dh_gazebo_imu_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "imuTopic").text = "imu"
        ET.SubElement(plugin, "gyroscopeNoiseDensity").text = f'{gyro_noise_density}'
        ET.SubElement(plugin, "gyroscopeRandomWalk").text = f'{gyro_random_walk}'
        ET.SubElement(plugin, "gyroscopeBiasCorrelationTime").text = f'{gyro_bias_corr_time}'
        ET.SubElement(plugin, "gyroscopeTurnOnBiasSigma").text = f'{gyro_turn_on_bias_sigma}'
        ET.SubElement(plugin, "accelerometerNoiseDensity").text = f'{acc_noise_density}'
        ET.SubElement(plugin, "accelerometerRandomWalk").text = f'{acc_random_walk}'
        ET.SubElement(plugin, "accelerometerBiasCorrelationTime").text = f'{acc_bias_corr_time}'
        ET.SubElement(plugin, "accelerometerTurnOnBiasSigma").text = f'{acc_turn_on_bias_sigma}'
