from xml.etree import ElementTree as ET
from typing import Tuple

from .sensor import SensorModel
from ..utils import all_ge


class OdometryModel(SensorModel):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        offset: Tuple[float, float, float],
        pos_normal_noise_std: Tuple[float, float, float],
        rot_normal_noise_std: Tuple[float, float, float],
        linvel_normal_noise_std: Tuple[float, float, float],
        angvel_normal_noise_std: Tuple[float, float, float],
        pos_uniform_noise_scale: Tuple[float, float, float],
        rot_uniform_noise_scale: Tuple[float, float, float],
        linvel_uniform_noise_scale: Tuple[float, float, float],
        angvel_uniform_noise_scale: Tuple[float, float, float],
    ):
        assert update_rate > 0.
        assert all_ge(pos_normal_noise_std, 0.)
        assert all_ge(rot_normal_noise_std, 0.)
        assert all_ge(linvel_normal_noise_std, 0.)
        assert all_ge(angvel_normal_noise_std, 0.)
        assert all_ge(pos_uniform_noise_scale, 0.)
        assert all_ge(rot_uniform_noise_scale, 0.)
        assert all_ge(linvel_uniform_noise_scale, 0.)
        assert all_ge(angvel_uniform_noise_scale, 0.)

        # 便宜的にセンサタイプをIMUにしている
        super().__init__(link_name, f'{ns}_barometer', "imu", update_rate)

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(self.sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_odometry_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_odometry_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "odometryTopic").text = "odometry"
        ET.SubElement(plugin, "offset").text = " ".join(map(str, offset))

        ET.SubElement(plugin, "noiseNormalPosition").text \
            = " ".join(map(str, pos_normal_noise_std))
        ET.SubElement(plugin, "noiseNormalRotation").text \
            = " ".join(map(str, rot_normal_noise_std))
        ET.SubElement(plugin, "noiseNormalLinearVelocity").text \
            = " ".join(map(str, linvel_normal_noise_std))
        ET.SubElement(plugin, "noiseNormalAngularVelocity").text \
            = " ".join(map(str, angvel_normal_noise_std))
        ET.SubElement(plugin, "noiseUniformPosition").text \
            = " ".join(map(str, pos_uniform_noise_scale))
        ET.SubElement(plugin, "noiseUniformRotation").text \
            = " ".join(map(str, rot_uniform_noise_scale))
        ET.SubElement(plugin, "noiseUniformLinearVelocity").text \
            = " ".join(map(str, linvel_uniform_noise_scale))
        ET.SubElement(plugin, "noiseUniformAngularVelocity").text \
            = " ".join(map(str, angvel_uniform_noise_scale))
