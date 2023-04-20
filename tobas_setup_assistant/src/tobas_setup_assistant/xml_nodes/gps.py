from xml.etree import ElementTree as ET


class GpsModel(ET.Element):

    def __init__(
        self,
        ns: str,
        link_name: str,
        update_rate: float,
        hor_pos_std: float,
        ver_pos_std: float,
        hor_vel_std: float,
        ver_vel_std: float,
        latitude_0: float,
        longitude_0: float,
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
        hor_pos_std : float
            水平位置のノイズの標準偏差
        ver_pos_std : float
            垂直位置のノイズの標準偏差
        hor_vel_std : float
            水平速度のノイズの標準偏差
        ver_vel_std : float
            垂直速度のノイズの標準偏差
        latitude_0 : float
            原点の緯度．北緯を正とする．
        longitude_0 : float
            原点の経度．東経を正とする．
        """
        assert update_rate > 0.
        assert hor_pos_std > 0.
        assert ver_pos_std > 0.
        assert hor_vel_std > 0.
        assert ver_vel_std > 0.
        assert -90. <= latitude_0 <= 90.
        assert -180 <= longitude_0 <= 180.

        # robot/gazebo
        super().__init__("gazebo", reference=link_name)

        # robot/gazebo/sensor
        sensor = ET.SubElement(self, "sensor")
        sensor.attrib["name"] = f'{ns}_gps'
        sensor.attrib["type"] = "gps"

        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = f'{update_rate}'
        ET.SubElement(sensor, "visualize").text = "false"
        ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"

        # robot/gazebo/sensor/plugin
        plugin = ET.SubElement(sensor, "plugin")
        plugin.attrib["filename"] = "libdh_gazebo_gps_plugin.so"
        plugin.attrib["name"] = "dh_gazebo_gps_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "gpsTopic").text = "gps"
        ET.SubElement(plugin, "groundSpeedTopic").text = "ground_speed"
        ET.SubElement(plugin, "horPosStdDev").text = f'{hor_pos_std}'
        ET.SubElement(plugin, "verPosStdDev").text = f'{ver_pos_std}'
        ET.SubElement(plugin, "horVelStdDev").text = f'{hor_vel_std}'
        ET.SubElement(plugin, "verVelStdDev").text = f'{ver_vel_std}'
        ET.SubElement(plugin, "latitudeZero").text = f'{latitude_0}'
        ET.SubElement(plugin, "longitudeZero").text = f'{longitude_0}'
