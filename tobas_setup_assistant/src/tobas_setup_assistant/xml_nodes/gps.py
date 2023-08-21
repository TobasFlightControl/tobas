from xml.etree import ElementTree as ET
from typing import Tuple

from .sensor import SensorModel


class GpsModel(SensorModel):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        delay: float,
        offset: Tuple[float, float, float],
        hor_pos_std: float,
        ver_pos_std: float,
        hor_vel_std: float,
        ver_vel_std: float,
        latitude_0: float,
        longitude_0: float,
        altitude_0: float,
    ) -> None:
        """
        GazeboのGPSセンサモデル．

        Parameters
        ----------
        ns : str
            namespace (ドローンの名前)
        link_name : str
            センサを取り付けるリンク名
        update_rate: float
            センサの更新頻度
        delay: float
            通信の遅延
        offset: Tuple[float, float, float]
            ルートリンクに対するGPSレシーバのオフセット．
        hor_pos_std : float
            水平位置のノイズの標準偏差
        ver_pos_std : float
            垂直位置のノイズの標準偏差
        hor_vel_std : float
            水平速度のノイズの標準偏差
        ver_vel_std : float
            垂直速度のノイズの標準偏差
        latitude_0 : float
            原点の緯度 [deg]．北緯を正とする．
        longitude_0 : float
            原点の経度 [deg]．東経を正とする．
        altitude_0: float
            原点の高度 [m]．上方を正とする．
        """
        assert update_rate > 0.
        assert delay >= 0.
        assert hor_pos_std > 0.
        assert ver_pos_std > 0.
        assert hor_vel_std > 0.
        assert ver_vel_std > 0.
        assert -90. <= latitude_0 <= 90.
        assert -180 <= longitude_0 <= 180.
        assert altitude_0 >= 0.

        super().__init__(link_name, f'{ns}_gps', "gps", 0.)  # プラグイン自体は毎周期呼ぶ

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(self.sensor, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_gps_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_gps_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "gpsTopic").text = "gps"
        ET.SubElement(plugin, "groundSpeedTopic").text = "ground_speed"
        ET.SubElement(plugin, "offset").text = " ".join(map(str, offset))
        ET.SubElement(plugin, "updateRate").text = str(update_rate)
        ET.SubElement(plugin, "delay").text = str(delay)
        ET.SubElement(plugin, "horPosStdDev").text = str(hor_pos_std)
        ET.SubElement(plugin, "verPosStdDev").text = str(ver_pos_std)
        ET.SubElement(plugin, "horVelStdDev").text = str(hor_vel_std)
        ET.SubElement(plugin, "verVelStdDev").text = str(ver_vel_std)
        ET.SubElement(plugin, "latitudeZero").text = str(latitude_0)
        ET.SubElement(plugin, "longitudeZero").text = str(longitude_0)
        ET.SubElement(plugin, "altitudeZero").text = str(altitude_0)
